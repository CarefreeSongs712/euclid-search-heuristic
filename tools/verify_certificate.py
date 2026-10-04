#!/usr/bin/env python3
"""Independently replay v11 certificates using mpmath, never the C++ solver.

Examples (both full interactive input and --threads/--solutions-shortened input):
  python tools/verify_certificate.py --input problem.in --certificate answer.json
  python tools/verify_certificate.py --certificate answer.json --precision=120 < problem.in
  python tools/verify_certificate.py --self-test

Only explicit given points and intersections of existing elements are available.
A circle's center and a bounded object's defining endpoints are NOT free points.
Exported doubles only identify witnesses; all constructions use matched, replayed
high-precision points. Lines use ax+by=c; certificate circles use c=radius**2,
whereas .in circles specify radius. Mode 3 has free integer grid lines/vertices.

Exit 0 means every supplied solution is legal and meets --tolerance (default
1e-8), or the original certificate EPS with --require-original-eps. A tolerance-
only success is explicitly PASS_TOLERANCE_ONLY, never an exact proof. Both goal
verdicts and residuals are always reported. Decimal .in tokens define the numeric
model; even 90-digit targets do NOT establish a generic/symbolic construction.
Geometry uses precision+20 working decimal digits. EPS and witness tolerance do
not create tangencies, intersections, or known points. Only working-roundoff-sized
slack is used for degeneracy/bounded-range arithmetic; the grid is strictly closed.
High-precision decimal strings are used in JSON output to avoid rounding to float.

Input layouts: full = E threads mode ... goals solutions; compact = E mode ...
goals; no-threads/no-solutions omit just that field. Auto accepts a unique parse,
otherwise --input-layout must disambiguate. No runtime third-party dependencies
other than mpmath. A missing mpmath installation is reported as a JSON failure.
"""

import argparse
import copy
from dataclasses import dataclass
import json
from pathlib import Path
import re
import sys

try:
    from mpmath import mp
except ImportError:
    mp = None


class VerificationError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def fail(code, message):
    raise VerificationError(code, message)


def number(value, label):
    # json.loads(parse_float=str) and direct mp.mpf(token) avoid binary64 parsing.
    if isinstance(value, bool) or not isinstance(value, (str, int)):
        fail("invalid_number", f"{label} must be a finite decimal number")
    text = str(value)
    if not re.fullmatch(r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?", text):
        fail("invalid_number", f"{label} is not a decimal number: {text!r}")
    result = mp.mpf(text)
    if not mp.isfinite(result):
        fail("invalid_number", f"{label} must be finite")
    return result


def vector(value, size, label):
    if not isinstance(value, list) or len(value) != size:
        fail("invalid_vector", f"{label} must contain {size} numbers")
    return tuple(number(v, f"{label}[{i}]") for i, v in enumerate(value))


def integer(value, label, maximum=None):
    if type(value) is not int or value < 0 or (maximum is not None and value > maximum):
        fail("invalid_integer", f"{label} must be a nonnegative integer" +
             (f" <= {maximum}" if maximum is not None else ""))
    return value


def inf_norm(values):
    return max((abs(x) for x in values), default=mp.mpf(0))


def differences(first, second):
    return tuple(abs(a - b) for a, b in zip(first, second))


def relative_error(first, second):
    # Componentwise scaling does not let a large y hide a bad x, or a large
    # circle radius hide an incorrect center.
    return max((abs(a - b) / max(1, abs(a), abs(b))
                for a, b in zip(first, second)), default=mp.mpf(0))


@dataclass(frozen=True)
class Element:
    kind: str
    coefficients: tuple
    bounds: tuple = ()


def line_from_points(first, second, kind="line"):
    x1, y1 = first
    x2, y2 = second
    if first == second:
        fail("degenerate_element", "A line/ray/segment needs two distinct points")
    return Element(kind, (y2 - y1, x1 - x2, x1 * y2 - y1 * x2),
                   (first, second) if kind in ("ray", "segment") else ())


def circle_from_points(first, second):
    radius_squared = sum((a - b) ** 2 for a, b in zip(first, second))
    if radius_squared == 0:
        fail("degenerate_element", "A paid circle needs two distinct points")
    return Element("circle", (*first, radius_squared))


class Tokens:
    def __init__(self, text):
        self.tokens = text.split()
        self.position = 0

    def take(self, label):
        if self.position >= len(self.tokens):
            fail("invalid_input", f"Missing {label}")
        token = self.tokens[self.position]
        self.position += 1
        return token

    def count(self, label, maximum=2147483647, minimum=0):
        token = self.take(label)
        if not re.fullmatch(r"\+?[0-9]+", token):
            fail("invalid_input", f"{label} must be an integer, got {token!r}")
        value = int(token)
        if value < minimum or value > maximum:
            fail("invalid_input", f"{label} outside {minimum}..{maximum}")
        return value

    def numbers(self, size, label):
        return tuple(number(self.take(label), label) for _ in range(size))


@dataclass
class Problem:
    budget: int
    mode: int
    grid: tuple
    points: list
    elements: list
    goal_points: list
    goal_elements: list
    layout: str


def point_in_grid(point, grid):
    return not grid or (0 <= point[0] <= grid[0] and 0 <= point[1] <= grid[1])


def parse_layout(text, layout):
    tokens = Tokens(text)
    budget = tokens.count("E", 65535)
    if layout in ("full", "no-solutions"):
        tokens.count("threads", 4294967295, 1)
    mode = tokens.count("mode", 3)
    grid = ()
    if mode == 3:
        grid = (tokens.count("grid m", 100000), tokens.count("grid n", 100000))
        if (grid[0] + 1) * (grid[1] + 1) > 1000000:
            fail("invalid_input", "Grid exceeds 1000000 vertices")
    counts = [tokens.count(f"given {kind}") for kind in ("P", "L", "R", "S", "C")]
    # Bounds work before allocating/iterating attacker-supplied enormous counts.
    if sum(n * size for n, size in zip(counts, (2, 3, 4, 4, 3))) > len(tokens.tokens) - tokens.position:
        fail("invalid_input", "Given counts exceed the available object data")
    points, elements = [], []

    def check_grid(point, label):
        if not point_in_grid(point, grid):
            fail("invalid_input", f"{label} is outside the closed grid")
        return point

    def read_line(label):
        coefficients = tokens.numbers(3, label)
        if coefficients[0] == 0 and coefficients[1] == 0:
            fail("invalid_input", f"{label} has a zero normal")
        return Element("line", coefficients)

    def read_circle(label):
        x, y, radius = tokens.numbers(3, label)
        if radius < 0:
            fail("invalid_input", f"{label} has negative radius")
        check_grid((x, y), f"{label} center")
        return Element("circle", (x, y, radius * radius))

    for _ in range(counts[0]):
        points.append(check_grid(tokens.numbers(2, "given point"), "Given point"))
    for _ in range(counts[1]):
        elements.append(read_line("given line"))
    for kind, count in zip(("ray", "segment"), counts[2:4]):
        for _ in range(count):
            first = check_grid(tokens.numbers(2, f"{kind} first"), kind)
            second = check_grid(tokens.numbers(2, f"{kind} second"), kind)
            elements.append(line_from_points(first, second, kind))
    for _ in range(counts[4]):
        elements.append(read_circle("given circle"))
    goals = [tokens.count(f"goal {kind}") for kind in ("L", "C", "P")]
    if not any(goals):
        fail("invalid_input", "At least one goal is required")
    if sum(n * size for n, size in zip(goals, (3, 3, 2))) > len(tokens.tokens) - tokens.position:
        fail("invalid_input", "Goal counts exceed the available object data")
    goal_elements = [read_line("goal line") for _ in range(goals[0])]
    goal_elements.extend(read_circle("goal circle") for _ in range(goals[1]))
    goal_points = [check_grid(tokens.numbers(2, "goal point"), "Goal point")
                   for _ in range(goals[2])]
    if layout in ("full", "no-threads"):
        tokens.count("solutions", 4294967295, 1)
    if tokens.position != len(tokens.tokens):
        fail("invalid_input", f"Unexpected trailing tokens in {layout} layout")
    return Problem(budget, mode, grid, points, elements, goal_points, goal_elements, layout)


def parse_input(text, layout="auto"):
    if layout != "auto":
        return parse_layout(text, layout)
    successes, errors = [], {}
    for candidate in ("full", "compact", "no-threads", "no-solutions"):
        try:
            successes.append(parse_layout(text, candidate))
        except VerificationError as exc:
            errors[candidate] = str(exc)
    if not successes:
        fail("invalid_input", "No supported .in layout parsed: " + json.dumps(errors))
    if len(successes) != 1:
        fail("ambiguous_input_layout", "Specify --input-layout; valid layouts: " +
             ", ".join(p.layout for p in successes))
    return successes[0]


def load_certificate(text):
    def no_constant(value):
        fail("invalid_number", f"Non-finite JSON constant {value} is forbidden")

    def unique_keys(items):
        result = {}
        for key, value in items:
            if key in result:
                fail("invalid_certificate", f"Duplicate JSON key: {key}")
            result[key] = value
        return result

    try:
        return json.loads(text, parse_float=str, parse_constant=no_constant,
                          object_pairs_hook=unique_keys)
    except json.JSONDecodeError as exc:
        fail("invalid_certificate", str(exc))


class Replay:
    def __init__(self, problem, precision, original_eps, tolerance, match_tolerance):
        self.problem = problem
        self.precision = precision
        self.original_eps = original_eps
        self.tolerance = tolerance
        self.match_tolerance = match_tolerance
        # Ten decimal guard digits beyond requested precision, plus another ten
        # in the working context. This is NOT the 1e-8 witness/goal tolerance.
        self.roundoff = mp.power(10, -(precision + 10))
        self.points = []
        self.births = []
        self.origins = []
        self.elements = []
        self.max_match_error = mp.mpf(0)
        self.max_relative_match_error = mp.mpf(0)
        self.max_element_error = mp.mpf(0)
        self.max_relative_element_error = mp.mpf(0)
        self.roundoff_tangencies = 0
        self.filtered_grid = 0
        self.filtered_bounded = 0
        self.multiple_matches = 0
        self.steps = []
        for index, point in enumerate(problem.points):
            self.add_point(point, 0, {"given_point": index})
        if problem.grid:
            m, n = problem.grid
            # These vertices really are intersections of the free grid lines.
            self.elements.extend(Element("line", (mp.mpf(1), mp.mpf(0), mp.mpf(x)))
                                 for x in range(m + 1))
            self.elements.extend(Element("line", (mp.mpf(0), mp.mpf(1), mp.mpf(y)))
                                 for y in range(n + 1))
            # Avoid O((mn)^2) deduplication of the automatic lattice itself.
            explicit = list(self.points)
            for y in range(n + 1):
                for x in range(m + 1):
                    point = (mp.mpf(x), mp.mpf(y))
                    if not any(relative_error(point, old) <= self.roundoff for old in explicit):
                        self.points.append(point)
                        self.births.append(0)
                        self.origins.append({"grid_intersection": [x, m + 1 + y]})
        for element in problem.elements:
            self.add_element(element, 0, paid=False)
        self.initial_point_count = len(self.points)
        self.initial_element_count = len(self.elements)

    def add_point(self, point, birth, origin):
        if not point_in_grid(point, self.problem.grid):
            self.filtered_grid += 1
            return
        if any(relative_error(point, old) <= self.roundoff for old in self.points):
            return
        self.points.append(point)
        self.births.append(birth)
        self.origins.append(origin)

    def unit_line(self, element):
        a, b, c = element.coefficients
        length = mp.sqrt(a * a + b * b)
        if length == 0:
            fail("degenerate_element", "Line has a zero normal")
        result = (a / length, b / length, c / length)
        # Orientation choice is immaterial; comparison tests both signs.
        return result

    def same_element(self, first, second):
        if first.kind != second.kind:
            return False
        if first.kind == "circle":
            return relative_error(first.coefficients, second.coefficients) <= self.roundoff
        a, b = self.unit_line(first), self.unit_line(second)
        if min(relative_error(a, b), relative_error(a, tuple(-x for x in b))) > self.roundoff:
            return False
        if first.kind == "line":
            return True
        p, q = first.bounds
        r, s = second.bounds
        same = lambda u, v: relative_error(u, v) <= self.roundoff
        if first.kind == "segment":
            return (same(p, r) and same(q, s)) or (same(p, s) and same(q, r))
        return same(p, r) and sum((q[i] - p[i]) * (s[i] - r[i]) for i in (0, 1)) > 0

    def in_range(self, element, point):
        if not element.bounds:
            return True
        first, second = element.bounds
        direction = tuple(b - a for a, b in zip(first, second))
        squared = sum(d * d for d in direction)
        parameter = sum((p - a) * d for p, a, d in zip(point, first, direction)) / squared
        slack = self.roundoff * max(1, abs(parameter))
        return parameter >= -slack and (element.kind == "ray" or parameter <= 1 + slack)

    def line_circle(self, line, circle):
        a, b, c = self.unit_line(line)
        x, y, radius_squared = circle.coefficients
        distance = a * x + b * y - c
        delta = radius_squared - distance * distance
        uncertainty = self.roundoff * max(1, abs(radius_squared), distance * distance)
        if delta < -uncertainty:
            return []
        foot = (x - a * distance, y - b * distance)
        if abs(delta) <= uncertainty:
            if delta != 0:
                self.roundoff_tangencies += 1
            return [foot]
        root = mp.sqrt(delta)
        return [(foot[0] - b * root, foot[1] + a * root),
                (foot[0] + b * root, foot[1] - a * root)]

    def intersections(self, first, second):
        if first.kind == "circle" and second.kind == "circle":
            x1, y1, r1 = first.coefficients
            x2, y2, r2 = second.coefficients
            if x1 == x2 and y1 == y2:
                return []  # Coincident or concentric circles reveal no point.
            radical = Element("line", (2 * (x1 - x2), 2 * (y1 - y2),
                                        (x1 - x2) * (x1 + x2) +
                                        (y1 - y2) * (y1 + y2) - r1 + r2))
            points = self.line_circle(radical, first)
        elif first.kind == "circle":
            points = self.line_circle(second, first)
        elif second.kind == "circle":
            points = self.line_circle(first, second)
        else:
            a, b, c = self.unit_line(first)
            d, e, f = self.unit_line(second)
            determinant = a * e - b * d
            if abs(determinant) <= self.roundoff:
                return []  # No arbitrary points on coincident supports.
            points = [((c * e - b * f) / determinant, (a * f - c * d) / determinant)]
        accepted = []
        for point in points:
            if self.in_range(first, point) and self.in_range(second, point):
                accepted.append(point)
            else:
                self.filtered_bounded += 1
        return accepted

    def add_element(self, element, birth, paid):
        if any(self.same_element(element, old) for old in self.elements):
            if paid:
                fail("repeated_element", "Paid step repeats an already existing element")
            return
        new_index = len(self.elements)
        for old_index, old in enumerate(self.elements):
            for point in self.intersections(element, old):
                self.add_point(point, birth, {"intersection": [old_index, new_index]})
        self.elements.append(element)

    def match(self, printed, label):
        candidates = []
        nearest = None
        for index, point in enumerate(self.points):
            relative = relative_error(printed, point)
            absolute = inf_norm(differences(printed, point))
            key = (relative, absolute, index)
            if nearest is None or key < nearest:
                nearest = key
            if relative <= self.match_tolerance:
                candidates.append(key)
        if not candidates:
            extra = "no known points" if nearest is None else (
                f"nearest relative error {mp.nstr(nearest[0], 12)}, "
                f"absolute error {mp.nstr(nearest[1], 12)}")
            fail("unknown_witness", f"{label} is not a known point ({extra})")
        relative, absolute, index = min(candidates)
        self.max_match_error = max(self.max_match_error, absolute)
        self.max_relative_match_error = max(self.max_relative_match_error, relative)
        if len(candidates) > 1:
            self.multiple_matches += 1
        return index, {
            "point_index": index, "birth_step": self.births[index],
            "origin": self.origins[index], "printed": printed,
            "matched": self.points[index], "absolute_error": absolute,
            "relative_error": relative, "candidates_within_tolerance": len(candidates),
        }

    def comparison_coefficients(self, actual, reference):
        """Compare lines in the reference's legacy b=1 / vertical a=1 scale.

        Unlike C++ CleanZero, never erase small geometric coefficients. Keeping
        b on an almost-vertical line avoids asserting exactness after EPS snaps.
        The pivot is chosen from the reference (.in or export), not a huge or
        tiny witness segment length. This retains source SameElement units for
        ordinary v11 lines; it is not a bit-for-bit binary64 replay.
        """
        if reference.kind == "circle":
            return actual.coefficients, reference.coefficients
        a, b, _ = reference.coefficients
        pivot = 1 if abs(b) >= self.original_eps else 0
        if reference.coefficients[pivot] == 0:
            pivot = 1 - pivot
        divisor = actual.coefficients[pivot]
        expected_divisor = reference.coefficients[pivot]
        expected = tuple(x / expected_divisor for x in reference.coefficients)
        if divisor == 0:
            return None, expected
        return tuple(x / divisor for x in actual.coefficients), expected

    def apply_step(self, step, index):
        if not isinstance(step, dict):
            fail("invalid_step", f"Step {index} must be an object")
        kind = step.get("type")
        if kind not in ("line", "circle"):
            fail("illegal_tool", f"Step {index}: only line/circle tools are legal")
        if (kind == "line" and self.problem.mode == 0) or (
                kind == "circle" and self.problem.mode in (1, 3)):
            fail("illegal_tool", f"Step {index}: {kind} is forbidden in mode {self.problem.mode}")
        first = vector(step.get("first"), 2, f"step {index} first")
        second = vector(step.get("second"), 2, f"step {index} second")
        exported = vector(step.get("element"), 3, f"step {index} element")
        first_index, first_report = self.match(first, f"Step {index} first")
        second_index, second_report = self.match(second, f"Step {index} second")
        if first_index == second_index:
            fail("degenerate_witness", f"Step {index} matches the same known point twice")
        p, q = self.points[first_index], self.points[second_index]
        actual = circle_from_points(p, q) if kind == "circle" else line_from_points(p, q)
        reference = Element(kind, exported)
        if kind == "line" and exported[0] == 0 and exported[1] == 0:
            fail("element_mismatch", f"Step {index}: exported line has zero normal")
        if kind == "circle" and exported[2] <= 0:
            fail("element_mismatch", f"Step {index}: paid circle needs positive radius squared")
        actual_values, expected_values = self.comparison_coefficients(actual, reference)
        if actual_values is None:
            fail("element_mismatch", f"Step {index}: exported line direction disagrees with its witnesses")
        absolute = inf_norm(differences(actual_values, expected_values))
        relative = relative_error(actual_values, expected_values)
        self.max_element_error = max(self.max_element_error, absolute)
        self.max_relative_element_error = max(self.max_relative_element_error, relative)
        if relative > self.match_tolerance:
            fail("element_mismatch", f"Step {index}: recomputed element disagrees with export "
                 f"(relative error {mp.nstr(relative, 12)})")
        before = len(self.points)
        self.add_element(actual, index, paid=True)
        self.steps.append({
            "step": index, "type": kind, "first": first_report, "second": second_report,
            "recomputed_element": actual_values, "exported_element": exported,
            "element_absolute_error": absolute, "element_relative_error": relative,
            "new_point_count": len(self.points) - before, "known_point_count": len(self.points),
        })

    def goal_report(self):
        goals = []
        for index, goal in enumerate(self.problem.goal_points):
            nearest = min(((inf_norm(differences(point, goal)), i, point)
                           for i, point in enumerate(self.points)), default=None)
            goals.append(self.goal_entry("point", index, goal, nearest))
        for index, goal in enumerate(self.problem.goal_elements):
            candidates = []
            _, expected = self.comparison_coefficients(goal, goal)
            for element_index, element in enumerate(self.elements):
                # A bounded object's carrier does not satisfy a line goal.
                if element.kind != goal.kind:
                    continue
                actual, _ = self.comparison_coefficients(element, goal)
                if actual is not None:
                    candidates.append((inf_norm(differences(actual, expected)), element_index, actual))
            nearest = min(candidates, default=None)
            goals.append(self.goal_entry(goal.kind, index, expected, nearest))
        return goals

    def goal_entry(self, kind, index, expected, nearest):
        if nearest is None:
            return {"type": kind, "goal_index": index, "target": expected,
                    "nearest_index": None, "residual": None, "component_residuals": None,
                    "original_eps_verdict": "FAIL", "tolerance_verdict": "FAIL"}
        residual, nearest_index, actual = nearest
        # This resolution check is conservative bookkeeping, not interval error
        # analysis. Large-scale/ill-conditioned problems can need more precision.
        resolution = mp.power(10, -self.precision) * max(1, inf_norm(expected), inf_norm(actual))
        verdict = lambda threshold: ("UNRESOLVED_AT_PRECISION" if threshold <= resolution else
                                     "PASS" if residual < threshold else "FAIL")
        return {
            "type": kind, "goal_index": index, "target": expected,
            "nearest_index": nearest_index, "nearest_value": actual,
            "residual": residual, "component_residuals": differences(actual, expected),
            "resolution_indicator": resolution,
            "original_eps_verdict": verdict(self.original_eps),
            "tolerance_verdict": verdict(self.tolerance),
        }

    def summary(self):
        return {"initial_point_count": self.initial_point_count,
                "initial_element_count": self.initial_element_count,
                "final_point_count": len(self.points), "final_element_count": len(self.elements),
                "max_match_error": self.max_match_error,
                "max_relative_match_error": self.max_relative_match_error,
                "max_element_error": self.max_element_error,
                "max_relative_element_error": self.max_relative_element_error,
                "multiple_candidate_witness_matches": self.multiple_matches,
                "roundoff_tangencies": self.roundoff_tangencies,
                "filtered_grid_intersections": self.filtered_grid,
                "filtered_bounded_intersections": self.filtered_bounded,
                "steps": self.steps}


def aggregate_verdict(verdicts):
    verdicts = list(verdicts)
    if "FAIL" in verdicts:
        return "FAIL"
    if "UNRESOLVED_AT_PRECISION" in verdicts:
        return "UNRESOLVED_AT_PRECISION"
    return "PASS"


def verify_solution(problem, solution, precision, eps, tolerance, match_tolerance, require_eps):
    replay = None
    result = {"legal": False, "passed": False, "status": "FAIL"}
    try:
        if not isinstance(solution, dict):
            fail("invalid_certificate", "Each solution must be an object")
        cost = integer(solution.get("E"), "solution E", 65535)
        steps = solution.get("steps")
        if not isinstance(steps, list):
            fail("invalid_certificate", "Solution steps must be an array")
        result.update({"E": cost, "step_count": len(steps), "budget": problem.budget})
        if cost != len(steps):
            fail("cost_mismatch", "Solution E must equal the number of paid steps")
        if cost > problem.budget:
            fail("budget_exceeded", "Solution exceeds the input E budget")
        replay = Replay(problem, precision, eps, tolerance, match_tolerance)
        for index, step in enumerate(steps, 1):
            result["checking_step"] = index
            replay.apply_step(step, index)
        result.pop("checking_step", None)
        goals = replay.goal_report()
        original = aggregate_verdict(g["original_eps_verdict"] for g in goals)
        tolerant = aggregate_verdict(g["tolerance_verdict"] for g in goals)
        passed = (original if require_eps else tolerant) == "PASS"
        result.update({"legal": True, "passed": passed, "goals": goals,
                       "original_eps_verdict": original, "tolerance_verdict": tolerant,
                       "status": ("PASS_ORIGINAL_EPS" if original == "PASS" else "PASS_TOLERANCE_ONLY")
                       if passed else "FAIL_GOALS"})
    except VerificationError as exc:
        result["error"] = {"code": exc.code, "message": str(exc)}
    if replay is not None:
        result.update(replay.summary())
    return result


def verify(problem, certificate, precision=80, tolerance="1e-8", match_tolerance="1e-8",
           require_original_eps=False):
    if not isinstance(certificate, dict) or certificate.get("version") != "v11":
        fail("invalid_certificate", "Certificate version must be v11")
    eps = number(certificate.get("eps"), "certificate eps")
    tolerance = number(tolerance, "tolerance")
    match_tolerance = number(match_tolerance, "match tolerance")
    if min(eps, tolerance, match_tolerance) <= 0:
        fail("invalid_tolerance", "EPS, tolerance, and match tolerance must be positive")
    solutions = certificate.get("solutions")
    if not isinstance(solutions, list) or not solutions:
        fail("empty_certificate", "Certificate must contain at least one solution")
    results = [verify_solution(problem, solution, precision, eps, tolerance,
                               match_tolerance, require_original_eps) for solution in solutions]
    for index, result in enumerate(results):
        result["solution_index"] = index
    passed = all(r["passed"] for r in results)
    original = aggregate_verdict(r.get("original_eps_verdict", "FAIL") for r in results)
    tolerant = aggregate_verdict(r.get("tolerance_verdict", "FAIL") for r in results)
    return {
        "verifier": "independent-mpmath-v11", "passed": passed,
        "status": ("PASS_ORIGINAL_EPS" if original == "PASS" else "PASS_TOLERANCE_ONLY")
        if passed else "FAIL", "exact_proof": False,
        "precision": precision, "working_precision": mp.dps,
        "numeric_model": "Input decimal tokens; recomputed high-precision geometry, not C++ binary64",
        "limitations": "Numerical instance verification only, not symbolic, interval-certified, or a generic proof",
        "goal_metric": "Strict absolute L-infinity: point coordinates; circle (cx,cy,r^2); line coefficients in target b=1/a=1 scale without EPS snapping",
        "witness_policy": "Nearest currently known high-precision point within componentwise scaled match tolerance; exported coordinates never become geometry",
        "range_policy": "Closed rays/segments with working-roundoff slack; strictly closed grid; no EPS-created tangencies",
        "acceptance": "original_eps" if require_original_eps else "tolerance",
        "original_eps": eps, "tolerance": tolerance, "match_tolerance": match_tolerance,
        "original_eps_verdict": original, "tolerance_verdict": tolerant,
        "input_layout": problem.layout, "mode": problem.mode, "budget": problem.budget,
        "solution_count": len(results),
        "max_match_error": max((r.get("max_match_error", mp.mpf(0)) for r in results)),
        "max_relative_match_error": max((r.get("max_relative_match_error", mp.mpf(0)) for r in results)),
        "solutions": results,
    }


def json_ready(value, digits):
    if isinstance(value, mp.mpf):
        return mp.nstr(value, digits)
    if isinstance(value, dict):
        return {key: json_ready(item, digits) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_ready(item, digits) for item in value]
    return value


def self_test(precision):
    """Small, deterministic in-memory certificates; no solver/search/subprocess."""
    def step(kind, first, second, element):
        return {"type": kind, "first": first, "second": second, "element": element}

    def cert(steps, eps="1e-11"):
        return {"version": "v11", "eps": eps, "solutions": [{"E": len(steps), "steps": steps}]}

    root = mp.nstr(mp.sqrt(3), 17)
    minus_root = "-" + root
    bisector_input = "3 2 2 0 0 0 0 0 0 2 0 1 0 0 1 0 1"
    bisector = cert([
        step("circle", [0, 0], [2, 0], [0, 0, 4]),
        step("circle", [2, 0], [0, 0], [2, 0, 4]),
        step("line", [1, root], [1, minus_root], [1, 0, 1]),
    ])
    tangent_input = "1 0 2 1 0 0 0 0 0 1 0 0 1 1 0 0 1 0 1"
    tangent = cert([step("circle", [0, 0], [1, 0], [0, 0, 1])])
    line_input = "1 1 2 0 0 0 0 0 0 2 0 1 0 0 0 1 0"
    horizontal = cert([step("line", [0, 0], [2, 0], [0, 1, 0])])
    grid_input = "2 3 1 1 0 0 0 0 0 0 0 1 .5 .5"
    grid = cert([step("line", [0, 0], [1, 1], [-1, 1, 0]),
                 step("line", [0, 1], [1, 0], [1, 1, 1])])
    cases = []

    def add(name, text, certificate, expected=True, error=None, **options):
        cases.append((name, text, certificate, expected, error, options))

    add("bisector_3E", bisector_input, bisector)
    add("full_stdin_layout", "3 1 " + bisector_input.split(" ", 1)[1] + " 1", bisector)
    add("no_threads_layout", bisector_input + " 1", bisector, layout="no-threads")
    add("no_solutions_layout", "3 1 " + bisector_input.split(" ", 1)[1], bisector, layout="no-solutions")
    add("compass_tangent_example", tangent_input, tangent)
    add("straightedge_mode1", line_input, horizontal)
    add("grid_diagonal_midpoint", grid_input, grid)
    # Initial intersections, not a free segment endpoint or circle center.
    add("initial_segment_intersection", "0 2 0 1 0 1 0 1 0 .5 0 0 1 0 0 0 1 .5 0", cert([]))
    add("initial_ray_forward_intersection", "0 2 0 1 1 0 0 1 0 2 0 0 1 0 0 0 1 2 0", cert([]))
    add("initial_circle_intersections", "0 2 0 1 0 0 1 0 1 0 0 0 1 0 0 1 1 0", cert([]))
    high_target = mp.nstr(mp.sqrt(3), 92)
    add("high_precision_target", "2 2 2 0 0 0 0 0 0 2 0 0 0 1 1 " + high_target,
        cert(bisector["solutions"][0]["steps"][:2]), tolerance="1e-60")
    add("unknown_goal_point_not_free", "1 2 1 0 0 0 0 0 0 0 0 1 1 0",
        cert([step("line", [0, 0], [1, 0], [0, 1, 0])]), False, "unknown_witness")
    add("given_circle_center_not_free", "1 2 1 0 0 0 1 1 0 0 0 1 1 0 0 0 1 0",
        cert([step("line", [0, 0], [1, 0], [0, 1, 0])]), False, "unknown_witness")
    add("segment_endpoints_not_free", "1 2 0 0 0 1 0 0 0 1 0 1 0 0 0 1 0",
        cert([step("line", [0, 0], [1, 0], [0, 1, 0])]), False, "unknown_witness")
    add("ray_endpoints_not_free", "1 2 0 0 1 0 0 0 0 1 0 1 0 0 0 1 0",
        cert([step("line", [0, 0], [1, 0], [0, 1, 0])]), False, "unknown_witness")
    add("ray_backward_intersection_filtered", "0 2 0 1 1 0 0 1 0 -1 0 0 1 0 0 0 1 -1 0", cert([]), False)
    add("segment_extension_filtered", "0 2 0 1 0 1 0 1 0 2 0 0 1 0 0 0 1 2 0", cert([]), False)
    add("bounded_carrier_is_not_a_line", "0 2 0 0 0 1 0 0 0 1 0 1 0 0 0 1 0", cert([]), False)
    add("circle_center_not_a_goal_point", "0 2 0 0 0 0 1 0 0 1 0 0 1 0 0", cert([]), False)
    add("compass_mode_forbids_lines", line_input.replace("1 1 ", "1 0 ", 1), horizontal,
        False, "illegal_tool")
    add("straightedge_mode_forbids_circles", tangent_input.replace("1 0 ", "1 1 ", 1), tangent,
        False, "illegal_tool")
    add("grid_mode_forbids_circles", grid_input,
        cert([step("circle", [0, 0], [1, 0], [0, 0, 1])]), False, "illegal_tool")
    add("grid_outside_intersections_not_witnesses", "1 3 1 1 0 1 0 0 0 1 0 2 1 0 0 0 1 0",
        cert([step("line", [2, 0], [0, 0], [0, 1, 0])]), False, "unknown_witness")
    add("future_intersection_not_witness", bisector_input,
        cert([bisector["solutions"][0]["steps"][2]]), False, "unknown_witness")
    add("budget_enforced", bisector_input.replace("3 ", "2 ", 1), bisector, False, "budget_exceeded")
    mismatch = copy.deepcopy(bisector)
    mismatch["solutions"][0]["E"] = 2
    add("declared_E_matches_paid_steps", bisector_input, mismatch, False, "cost_mismatch")
    wrong_element = copy.deepcopy(bisector)
    wrong_element["solutions"][0]["steps"][0]["element"] = [0, 0, 2]
    add("circle_export_is_radius_squared", bisector_input, wrong_element, False, "element_mismatch")
    wrong_line = copy.deepcopy(horizontal)
    wrong_line["solutions"][0]["steps"][0]["element"] = [1, 0, 0]
    add("exported_line_checked", line_input, wrong_line, False, "element_mismatch")
    add("same_witness_twice", line_input, cert([step("line", [0, 0], [0, 0], [0, 1, 0])]),
        False, "degenerate_witness")
    add("illegal_step_type", line_input, cert([step("segment", [0, 0], [2, 0], [0, 1, 0])]),
        False, "illegal_tool")
    add("no_approximate_phantom_tangent", "0 2 0 1 0 0 1 0 1 1.0000000000001 0 0 1 0 0 1 0 1", cert([]), False)
    add("no_paid_duplicate", line_input.replace("1 1 ", "2 1 ", 1),
        cert(horizontal["solutions"][0]["steps"] * 2), False, "repeated_element")
    add("empty_solutions_rejected", line_input, {"version": "v11", "eps": "1e-11", "solutions": []},
        False, "empty_certificate")
    add("nonfinite_rejected", line_input,
        cert([step("line", ["NaN", 0], [2, 0], [0, 1, 0])]), False, "invalid_number")
    almost_input = "0 2 1 0 0 0 0 0 0 0 0 1 .000000005 0"
    add("tolerance_only_explicitly_labeled", almost_input, cert([]), expected_status="PASS_TOLERANCE_ONLY")
    add("original_eps_required", almost_input, cert([]), False, require_original_eps=True)
    # Printed perturbations must never become real geometry or goal points.
    perturbed = cert([step("line", [0, "5e-9"], [2, "5e-9"], [0, 1, "5e-9"])])
    add("exported_coordinates_not_used_as_geometry", line_input, perturbed,
        require_original_eps=True, expect_nonzero_match=True)
    add("insufficient_precision_not_claimed", "0 2 1 0 0 0 0 0 0 0 0 1 0 0", cert([], "1e-200"),
        False, require_original_eps=True)
    reports = []
    for name, text, certificate, expected, error, options in cases:
        options = dict(options)
        try:
            layout = options.pop("layout", "compact")
            if name == "full_stdin_layout":
                layout = "auto"
            expected_status = options.pop("expected_status", None)
            nonzero_match = options.pop("expect_nonzero_match", False)
            problem = parse_input(text, layout)
            result = verify(problem, certificate, precision=precision, **options)
            actual_error = next((r.get("error", {}).get("code") for r in result["solutions"]
                                 if r.get("error")), None)
            passed = result["passed"] == expected and (error is None or actual_error == error)
            if expected_status:
                passed = passed and result["status"] == expected_status
            if nonzero_match:
                passed = passed and result["max_match_error"] > 0
            report = {"name": name, "passed": passed, "verification_status": result["status"]}
            if actual_error:
                report["error_code"] = actual_error
            if not passed:
                report["details"] = result
            reports.append(report)
        except VerificationError as exc:
            passed = not expected and error == exc.code
            reports.append({"name": name, "passed": passed, "error_code": exc.code, "message": str(exc)})
    return {"self_test": True, "passed": all(r["passed"] for r in reports),
            "test_count": len(reports), "failed_count": sum(not r["passed"] for r in reports),
            "tests": reports}


class JsonArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        fail("invalid_arguments", message)


def main(argv=None):
    parser = JsonArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", type=Path, help="Original .in file (default: stdin)")
    parser.add_argument("--certificate", type=Path, help="v11 --certificate JSON file")
    parser.add_argument("--input-layout", choices=("auto", "full", "compact", "no-threads", "no-solutions"),
                        default="auto")
    parser.add_argument("--precision", type=int, default=80, help="Requested decimal precision, >=30 (default: 80)")
    parser.add_argument("--tolerance", default="1e-8", help="Absolute goal tolerance (default: 1e-8)")
    parser.add_argument("--match-tolerance", default="1e-8", help="Componentwise scaled export-matching tolerance")
    parser.add_argument("--require-original-eps", action="store_true", help="Exit 1 unless original EPS goals pass")
    parser.add_argument("--self-test", action="store_true", help="Run small positive/negative tests without searching")
    digits = 80
    try:
        args = parser.parse_args(argv)
        if mp is None:
            fail("missing_dependency", "mpmath is required; install with: python -m pip install mpmath")
        if args.precision < 30 or args.precision > 10000:
            fail("invalid_precision", "--precision must be in 30..10000 decimal digits")
        digits = args.precision
        with mp.workdps(digits + 20):
            if args.self_test:
                if args.input or args.certificate:
                    fail("invalid_arguments", "--self-test cannot be combined with input/certificate paths")
                report = self_test(digits)
            else:
                if args.certificate is None:
                    fail("invalid_arguments", "--certificate is required unless --self-test is used")
                text = args.input.read_text(encoding="utf-8-sig") if args.input else sys.stdin.read().lstrip("\ufeff")
                problem = parse_input(text, args.input_layout)
                certificate = load_certificate(args.certificate.read_text(encoding="utf-8-sig"))
                report = verify(problem, certificate, digits, args.tolerance, args.match_tolerance,
                                args.require_original_eps)
                report["input"] = str(args.input.resolve()) if args.input else "stdin"
                report["certificate"] = str(args.certificate.resolve())
            output = json_ready(report, digits)
    except (VerificationError, OSError, UnicodeError, ValueError, OverflowError) as exc:
        output = {"passed": False, "status": "FAIL", "error": {
            "code": getattr(exc, "code", "execution_error"), "message": str(exc)}}
    print(json.dumps(output, ensure_ascii=True, indent=2, allow_nan=False))
    return 0 if output["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
