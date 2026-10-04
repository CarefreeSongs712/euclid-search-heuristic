#include "solver.hpp"
#include "heuristic.hpp"
#include "certificate.hpp"
#include "reporting.hpp"
#include "progress.hpp"
#include <memory>
#include <filesystem>
#include <string_view>
#ifdef _WIN32
#include <io.h>
#else
#include <unistd.h>
#endif

using namespace bs;

static string DetectOperatingSystem() {
#ifdef _WIN32
    return "Windows";
#elif defined(__linux__)
    ifstream osRelease("/etc/os-release");
    string line;
    while (getline(osRelease, line)) {
        if (line == "ID=ubuntu" || line == "ID=\"ubuntu\"") return "Ubuntu";
    }
    return "Linux";
#else
    return "Unknown OS";
#endif
}

#ifdef __linux__
static uint32_t CountCpuList(const string& text) {
    uint32_t count = 0;
    size_t pos = 0;
    while (pos < text.size()) {
        while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n' || text[pos] == ',')) ++pos;
        if (pos >= text.size()) break;
        char* endp = nullptr;
        const unsigned long first = strtoul(text.c_str() + pos, &endp, 10);
        if (endp == text.c_str() + pos) break;
        pos = static_cast<size_t>(endp - text.c_str());
        unsigned long last = first;
        if (pos < text.size() && text[pos] == '-') {
            ++pos;
            last = strtoul(text.c_str() + pos, &endp, 10);
            if (endp == text.c_str() + pos) break;
            pos = static_cast<size_t>(endp - text.c_str());
        }
        if (last >= first) count += static_cast<uint32_t>(last - first + 1);
        while (pos < text.size() && text[pos] != ',') ++pos;
        if (pos < text.size()) ++pos;
    }
    return count;
}
#endif

static uint32_t DetectAvailableThreadLimit(uint32_t hardwareThreads) {
    uint32_t available = max<uint32_t>(1, hardwareThreads);
#ifdef __linux__
    {
        ifstream f("/sys/fs/cgroup/cpu.max");
        string quotaText;
        uint64_t period = 0;
        if (f >> quotaText >> period && quotaText != "max" && period > 0) {
            const uint64_t quota = strtoull(quotaText.c_str(), nullptr, 10);
            const uint32_t quotaThreads = static_cast<uint32_t>(max<uint64_t>(1, (quota + period - 1) / period));
            available = min(available, quotaThreads);
        }
    }
    {
        ifstream f("/sys/fs/cgroup/cpuset.cpus.effective");
        string cpus;
        if (getline(f, cpus)) {
            const uint32_t n = CountCpuList(cpus);
            if (n > 0) available = min(available, n);
        }
    }
#endif
    return max<uint32_t>(1, available);
}

static int ExitWithPause(int code, bool pauseOnExit) {
#ifdef _WIN32
    const bool inputIsTerminal = _isatty(_fileno(stdin)) != 0;
#else
    const bool inputIsTerminal = isatty(STDIN_FILENO) != 0;
#endif
    // Opt-in only, and never hang a pipe/redirected-input invocation.
    if (pauseOnExit && inputIsTerminal) {
        cerr << "\nPress Enter to close this window..." << flush;
        cin.clear();
        if (cin.rdbuf()->in_avail() > 0)
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cin.get();
    }
    return code;
}

static uint64_t ParseUnsignedOption(string_view text, const char* name,
                                    uint64_t minimum, uint64_t maximum) {
    uint64_t value = 0;
    const auto parsed = from_chars(text.data(), text.data() + text.size(), value);
    if (text.empty() || parsed.ec != errc{} || parsed.ptr != text.data() + text.size() ||
        value < minimum || value > maximum)
        throw invalid_argument(string(name) + " must be an integer in " +
                               to_string(minimum) + ".." + to_string(maximum) + '.');
    return value;
}

static double ParsePositiveOption(string_view text, const char* name,
                                  double minimum, double maximum) {
    double value = 0.0;
    size_t consumed = 0;
    // MinGW GCC 11 lacks the floating-point from_chars overloads. stod is
    // compatible there; catch only its numeric conversion exceptions, never
    // bad_alloc, and reject whitespace, suffixes and non-finite results.
    bool valid = !text.empty() && text.find_first_of(" \t\r\n\f\v") == string_view::npos;
    if (valid) {
        try { value = stod(string(text), &consumed); }
        catch (const invalid_argument&) { valid = false; }
        catch (const out_of_range&) { valid = false; }
    }
    if (!valid || consumed != text.size() || !isfinite(value) ||
        !(value > 0.0) || value < minimum || value > maximum)
        throw invalid_argument(string(name) + " must be a positive finite number in the supported range (no trailing characters).");
    return value;
}

// Formatted integer extraction accepts prefixes such as "1junk". Tokenize the
// integer fields instead, preserving their original whitespace-based layout.
// Geometry coordinates deliberately retain the original floating-point parser.
template <class Integer>
static bool ReadInteger(Integer& value) {
    string text;
    if (!(cin >> text) || text.empty()) return false;
    string_view digits(text);
    if (digits.front() == '+') digits.remove_prefix(1);
    if (digits.empty()) return false;
    const auto parsed = from_chars(digits.data(), digits.data() + digits.size(), value);
    return parsed.ec == errc{} && parsed.ptr == digits.data() + digits.size();
}

static void PrintHelp() {
    cout << R"HELP(brute_search v11-r3.2 heuristic
Usage: brute_search [OPTIONS] < input.txt

The legacy whitespace-separated stdin format is unchanged:
  E; worker threads (unless --threads=N); tool type (0/1/2/3);
  grid m n for tool type 3; given counts P L R S C and object data;
  goal counts L C P and goal data; solutions (unless --solutions=N).
E must be in 0..65535. Geometry coordinates/coefficients retain legacy syntax.

General:
  --help, -h                Show this help without reading stdin
  --version                Show version without reading stdin
  --threads=N              Worker threads, 1..process-available maximum
  --solutions=N            Requested distinct solutions, 1..4294967295
  --time-limit=N           Positive finite seconds (default: 30)
  --eps=N                  Positive finite geometry tolerance (default: 1e-11)
  --pause / --no-pause      Opt-in terminal-only exit pause (default: no pause)

Progress (stderr only; result/stdout format is unchanged):
  --progress-interval=N    Sampling interval in seconds, >=0.001 (default: 2)
  --no-progress            Disable sampling and progress output
  --progress               Enable progress output
  E, elapsed, nodes/rate, raw/unique candidates, threads and completed
  prefixes are shown. Exhaustion ETA is unknown during planning/warmup.
  Later estimates are explicitly rough/workload-biased: root fraction in
  single-thread search, completed/total prefixes only after the producer
  exhausts its frontier in parallel search. Heuristic mode has no exhaustion
  fraction/ETA. Timeout budget is NOT an ETA.

Heuristic engine (default):
  --search=heuristic|exhaustive  v11 beam search or retained v10 DFS
  --prerequisites / --no-prerequisites  Paid diameter/homothety preparation and
                                witnessed goal-element finishing (default on)
  --structural / --no-structural  r3 paid-scaffold / point / dependency-chain joins
                                Default on, bounded local slices; never free macros
  --landmarks / --no-landmarks Experimental backward geometric scoring (off)
  --adaptive / --no-adaptive   r2 structural rendezvous and diversity portfolio
                                Default on. A helper-enabled parallel run reserves
                                one continuing DFS lane after the first beam pass.
  --coverage-threads=auto|N     Shared-prefix DFS slots, incl. producer
                                Auto: half, or quarter for E>=8; at least 2.
                                0 disables; explicit count 2..threads-1.
                                Enabled only with >=4 threads, unlimited restarts,
                                adaptive and tail helpers on; total threads unchanged.
  --beam-width=N                Retained states per worker (default: 64)
  --branch-limit=N              Evaluated successors per parent (default: 96)
  --restarts=N                  Portfolio attempts (0 = until deadline)
  --seed=N                      Seed for deterministic random streams
                                (wall-clock cutoffs can still change results)
  --tail-seconds=N              Time per short legacy tail helper (default: .02)
  --certificate=FILE           Export verified 17-digit JSON construction witnesses
                                (new file only; never overwrites an existing file)
  --tail-candidates=N           Tail helpers per layer (default: 4; 0 disables)
  Beam/candidate caps trade coverage for speed. No solution is NOT proof of
  impossibility. Heuristic mode never returns EXHAUSTED or an exhaustion ETA.

Search/output (legacy options; search controls below apply to exhaustive mode):
  --symmetry / --no-symmetry             Default: symmetry enabled
  --no-goal-first                        Disable exact-goal-first ordering
  --low-memory / --dedup-candidates      Default: low-memory streaming
  --tt-mb=N / --no-tt                    TT memory MiB (default: 8)
  --stream-dedup=N / --no-stream-dedup    Cache entries (default: 2048)
  --task-depth=auto|N                    Frontier depth, 1..65535 (default: auto)
  --grid-fast / --no-grid-fast           Default: grid fast path enabled
  --readable-output / --raw-output       Default: readable solution output

Numbers must consume the entire option value; NaN, infinity and overflow
are rejected. Time values must also fit the monotonic clock's range.
Exit codes: 0 = found, 2 = exhausted without a solution, 3 = timeout
(including partial solutions), 4 = heuristic attempts stopped without quota,
1 = invalid input or execution failure.
)HELP";
}

static bool ParseSolutionCount(const string& text, size_t& value) {
    if (text.empty()) return false;
    uint32_t count = 0;
    const auto result = from_chars(text.data(), text.data() + text.size(), count);
    if (result.ec != errc{} || result.ptr != text.data() + text.size() || count == 0)
        return false;
    value = count;
    return true;
}

static int Run(int argc, char** argv, bool& pauseOnExit) {
    bool heuristic = true;
    HeuristicOptions heuristicOptions;
    string certificatePath;
    bool symmetry = true;
    bool goalFirst = true;
    bool lowMemory = true;
    size_t ttMB = 8;
    size_t streamDedupEntries = 2048;
    double timeLimitSeconds = DEFAULT_TIME_LIMIT_SECONDS;
    double progressInterval = 2.0;
    bool progressEnabled = true;
    uint16_t requestedTaskDepth = 0; // 0 = adaptive default from thread count
    uint32_t requestedThreads = 0; // 0 = ask interactively
    bool readableOutput = true;
    bool gridFast = true;
    size_t requestedSolutions = 0; // 0 = ask after all goal data

    // Informational invocations never parse stdin, construct a solver or pause.
    for (int i = 1; i < argc; ++i) {
        if (string_view(argv[i]) == "--help" || string_view(argv[i]) == "-h") {
            PrintHelp();
            return 0;
        }
    }
    for (int i = 1; i < argc; ++i) {
        if (string_view(argv[i]) == "--version") {
            cout << "brute_search v11-r3.2 heuristic\n";
            return 0;
        }
    }

    // Leave ample clock headroom for steady_clock::now() + duration and for
    // duration<double> conversion on all supported clock representations.
    const double maximumClockSeconds = chrono::duration<double>(
        chrono::steady_clock::duration::max()).count() / 4.0;
    const double minimumClockSeconds = chrono::duration<double>(
        chrono::steady_clock::duration(1)).count();
    constexpr uint64_t mib = 1024ULL * 1024ULL;
    // The bounded dedup table rounds 2*N up to a power of two. Keep its worst
    // case allocation arithmetic representable; allocation failure is distinct
    // from invalid numeric syntax and is handled by the top-level handler.
    constexpr uint64_t maxDedupEntries = numeric_limits<size_t>::max() /
        (4ULL * (sizeof(Element) + 16ULL));
    for (int i = 1; i < argc; ++i) {
        string_view arg = argv[i];
        if (arg.starts_with("--search=")) {
            const auto mode = arg.substr(9);
            if (mode != "heuristic" && mode != "exhaustive")
                throw invalid_argument("Search must be heuristic or exhaustive.");
            heuristic = mode == "heuristic";
        }
        else if (arg.starts_with("--certificate=")) {
            certificatePath = string(arg.substr(14));
            if (certificatePath.empty()) throw invalid_argument("Certificate path must not be empty.");
        }
        else if (arg == "--prerequisites") heuristicOptions.prerequisites = true;
        else if (arg == "--no-prerequisites") heuristicOptions.prerequisites = false;
        else if (arg == "--structural") heuristicOptions.structural = true;
        else if (arg == "--no-structural") heuristicOptions.structural = false;
        else if (arg == "--landmarks") heuristicOptions.landmarks = true;
        else if (arg == "--no-landmarks") heuristicOptions.landmarks = false;
        else if (arg == "--adaptive") heuristicOptions.adaptive = true;
        else if (arg == "--no-adaptive") heuristicOptions.adaptive = false;
        else if (arg.starts_with("--coverage-threads=")) {
            const auto text=arg.substr(19);
            heuristicOptions.coverageThreads=text=="auto"?-1:static_cast<int>(
                ParseUnsignedOption(text,"Coverage threads",0,numeric_limits<int>::max()));
        }
        else if (arg.starts_with("--beam-width="))
            heuristicOptions.beamWidth = ParseUnsignedOption(arg.substr(13), "Beam width per worker", 1, 16384);
        else if (arg.starts_with("--branch-limit="))
            heuristicOptions.branchLimit = ParseUnsignedOption(arg.substr(15), "Branch limit per parent", 1, 16384);
        else if (arg.starts_with("--restarts="))
            heuristicOptions.restarts = ParseUnsignedOption(arg.substr(11), "Restarts", 0, 1000000000);
        else if (arg.starts_with("--seed="))
            heuristicOptions.seed = ParseUnsignedOption(arg.substr(7), "Random seed", 0, numeric_limits<uint64_t>::max());
        else if (arg.starts_with("--tail-seconds="))
            heuristicOptions.tailSeconds = arg.substr(15) == "0" ? 0.0 :
                ParsePositiveOption(arg.substr(15), "Tail search seconds", .001, 60);
        else if (arg.starts_with("--tail-candidates="))
            heuristicOptions.tailCandidates = ParseUnsignedOption(arg.substr(18), "Tail candidates per layer", 0, 256);
        else if (arg == "--symmetry") symmetry = true;
        else if (arg == "--no-symmetry") symmetry = false;
        else if (arg == "--no-goal-first") goalFirst = false;
        else if (arg == "--low-memory") lowMemory = true;
        else if (arg == "--dedup-candidates") lowMemory = false;
        else if (arg == "--no-tt") ttMB = 0;
        else if (arg.starts_with("--tt-mb="))
            ttMB = static_cast<size_t>(ParseUnsignedOption(arg.substr(8), "TT MiB", 0,
                numeric_limits<size_t>::max() / mib));
        else if (arg == "--no-stream-dedup") streamDedupEntries = 0;
        else if (arg.starts_with("--stream-dedup="))
            streamDedupEntries = static_cast<size_t>(ParseUnsignedOption(
                arg.substr(15), "Stream dedup entries", 0, maxDedupEntries));
        else if (arg.starts_with("--eps="))
            EPS = ParsePositiveOption(arg.substr(6), "EPS", 0.0, numeric_limits<double>::max());
        else if (arg.starts_with("--time-limit="))
            timeLimitSeconds = ParsePositiveOption(arg.substr(13), "Time limit",
                minimumClockSeconds, maximumClockSeconds);
        else if (arg.starts_with("--progress-interval="))
            progressInterval = ParsePositiveOption(arg.substr(20), "Progress interval",
                max(0.001, minimumClockSeconds), maximumClockSeconds);
        else if (arg == "--no-progress") progressEnabled = false;
        else if (arg == "--progress") progressEnabled = true;
        else if (arg.starts_with("--task-depth=")) {
            const auto text = arg.substr(13);
            requestedTaskDepth = text == "auto" ? 0 : static_cast<uint16_t>(
                ParseUnsignedOption(text, "Task depth ('auto' or integer)", 1, 65535));
        }
        else if (arg.starts_with("--threads="))
            requestedThreads = static_cast<uint32_t>(ParseUnsignedOption(arg.substr(10),
                "Thread count", 1, numeric_limits<uint32_t>::max()));
        else if (arg.starts_with("--solutions="))
            requestedSolutions = static_cast<size_t>(ParseUnsignedOption(arg.substr(12),
                "Solution count", 1, numeric_limits<uint32_t>::max()));
        else if (arg == "--no-grid-fast") gridFast = false;
        else if (arg == "--grid-fast") gridFast = true;
        else if (arg == "--raw-output") readableOutput = false;
        else if (arg == "--readable-output") readableOutput = true;
        else if (arg == "--pause") pauseOnExit = true;
        else if (arg == "--no-pause") pauseOnExit = false;
        else throw invalid_argument("Unknown command-line option: " + string(arg));
    }

    int limit = 0;
    int toolType = 2;
    int selectedMode = 2;
    array<int, 6> initial{};
    int goalLines = 0, goalCircles = 0, goalPoints = 0;
    Graph graph;
    const string operatingSystem = DetectOperatingSystem();
    uint32_t hardwareThreads = thread::hardware_concurrency();
    if (hardwareThreads == 0) hardwareThreads = 1;
    const uint32_t availableThreads = DetectAvailableThreadLimit(hardwareThreads);
    if (requestedThreads > availableThreads)
        throw invalid_argument("Thread count exceeds process-available maximum " +
                               to_string(availableThreads) + '.');

    cout << "============================================================\n";
    cout << "brute_search v11-r3.2 heuristic\n";
    cout << "Geometry EPS: " << scientific << setprecision(3) << EPS << defaultfloat << "\n";
    cout << "Search time limit: " << timeLimitSeconds << " seconds\n";
    cout << "Detected system: " << operatingSystem << "\n";
    cout << "Hardware logical processors: " << hardwareThreads << "\n";
    cout << "Process-available / recommended maximum threads: "
         << availableThreads << "\n";
#ifdef _WIN32
    if (pauseOnExit)
        cout << "";
#endif
    cout << "============================================================\n\n";

    cout << "[Step 1/6] Maximum number of NEW construction elements (E).\n";
    cout << "  E counts only elements constructed by the solver; given lines/circles do NOT count.\n";
    cout << "  Example: enter 7 to search for a construction using at most 7 new lines/circles.\n";
    cout << "Enter E limit: " << flush;
    if (!ReadInteger(limit) || limit < 0 || limit > 65535) {
        cerr << "Invalid E limit. Please enter an integer in 0..65535.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    uint32_t threads = requestedThreads;
    if (threads == 0) {
        cout << "\n[Step 2/6] Select parallel worker threads.\n";
        cout << "  Detected " << operatingSystem << ": " << hardwareThreads
             << " hardware logical processor(s), " << availableThreads
             << " available to this process.\n";
        cout << "  Recommended maximum: " << availableThreads
             << " thread(s).  Use 1 for deterministic single-thread mode.\n";
        cout << "Enter worker threads [1-" << availableThreads << "]: " << flush;
        if (!ReadInteger(threads)) {
            cerr << "Invalid thread count.\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }
    if (threads < 1 || threads > availableThreads) {
        cerr << "Invalid thread count. Please enter 1.." << availableThreads
             << " for this " << operatingSystem << " system.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    cout << "\n[Step 3/6] Select allowed construction tools.\n";
    cout << "  0 = compass only      (construct circles from two known points)\n";
    cout << "  1 = straightedge only (construct lines through two known points)\n";
    cout << "  2 = both compass and straightedge\n";
    cout << "  3 = grid straightedge only\n";
    cout << "Enter tool type [0/1/2/3]: " << flush;
    if (!ReadInteger(selectedMode) || selectedMode < 0 || selectedMode > 3) {
        cerr << "Invalid tool type. Please enter 0, 1, 2, or 3.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    toolType = selectedMode == 3 ? 1 : selectedMode;
    if (selectedMode == 3) {
        cout << "网格左下角=(0,0)，右上角=(m,n)。请输入非负整数 m n：" << flush;
        long long m, n;
        if (!ReadInteger(m) || !ReadInteger(n) || m < 0 || n < 0 || m > 100000 || n > 100000 ||
            (m + 1) * (n + 1) > 1000000) {
            cerr << "Invalid grid: m,n must be nonnegative integers <=100000, "
                    "with at most 1000000 grid vertices.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.gridMode = true;
        graph.gridFast = gridFast;
        graph.gridM = static_cast<int>(m);
        graph.gridN = static_cast<int>(n);
        cout << "自动添加 x=0..m、y=0..n 的网格线及其网格内交点，不计入E。\n";
    }

    cout << "\n[Step 4/6] Enter counts of GIVEN objects in this exact order:\n";
    cout << "  P L R S C\n";
    cout << "  P = points, L = infinite lines, R = rays, S = segments, C = circles.\n";
    cout << "  Example: '2 1 0 0 0' means 2 points and 1 line are initially given.\n";
    cout << "Enter P L R S C: " << flush;
    if (!ReadInteger(initial[1]) || !ReadInteger(initial[2]) || !ReadInteger(initial[3]) ||
        !ReadInteger(initial[4]) || !ReadInteger(initial[5])) {
        cerr << "Invalid initial-object counts.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    for (int k = 1; k <= 5; ++k) {
        if (initial[k] < 0) {
            cerr << "Initial-object counts must be non-negative.\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }

    size_t totalElementCapacity = static_cast<size_t>(limit) + 4;
    for (int k = 2; k <= 5; ++k) {
        const size_t count = static_cast<size_t>(initial[k]);
        if (count > numeric_limits<size_t>::max() - totalElementCapacity)
            throw invalid_argument("Initial element capacity overflow.");
        totalElementCapacity += count;
    }
    const size_t gridElements = graph.gridMode
        ? static_cast<size_t>(graph.gridM) + static_cast<size_t>(graph.gridN) + 2 : 0;
    if (gridElements > numeric_limits<size_t>::max() - totalElementCapacity)
        throw invalid_argument("Grid element capacity overflow.");
    totalElementCapacity += gridElements;
    // Reserve is only a growth hint, not a search limit. Geometry's legacy
    // hint is quadratic in E; cap eager allocation so a valid E=65535 does not
    // allocate billions of points before even reading the given geometry.
    // Vectors still grow normally as actual input/search work requires.
    graph.Reserve(min<size_t>(totalElementCapacity, 1024));

    for (int i = 0; i < initial[1]; ++i) {
        double x, y;
        cout << "\nGiven point P" << i + 1 << ": enter x y.\n";
        cout << "  Example: '0 0' means P" << i + 1 << "=(0,0).\n";
        cout << "P" << i + 1 << " = " << flush;
        if (!(cin >> x >> y)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x, y})) {
            cerr << "Given point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddPoint({x, y}, 0);
    }

    const size_t givenPointCount = graph.points.size();
    if (graph.gridMode) graph.AddAutomaticGridLines();

    for (int i = 0; i < initial[2]; ++i) {
        double a, b, c;
        cout << "\nGiven line L" << i + 1 << ": enter a b c for a*x + b*y = c.\n";
        cout << "  Examples: y=0 -> '0 1 0'; x=2 -> '1 0 2'.\n";
        cout << "L" << i + 1 << " (a b c) = " << flush;
        if (!(cin >> a >> b >> c)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        graph.AddInitial(Element::FromCoefficients(a, b, c, Type::Line));
    }

    for (int i = 0; i < initial[3]; ++i) {
        double x1, y1, x2, y2;
        cout << "\nGiven ray R" << i + 1 << ": enter x1 y1 x2 y2.\n";
        cout << "  The ray starts at (x1,y1) and passes through (x2,y2).\n";
        cout << "R" << i + 1 << " = " << flush;
        if (!(cin >> x1 >> y1 >> x2 >> y2)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x1, y1}) || !graph.PointAllowed({x2, y2})) {
            cerr << "Given ray/segment defining point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddInitialBounded({x1, y1}, {x2, y2}, Type::Ray);
    }

    for (int i = 0; i < initial[4]; ++i) {
        double x1, y1, x2, y2;
        cout << "\nGiven segment S" << i + 1 << ": enter x1 y1 x2 y2 for its two endpoints.\n";
        cout << "S" << i + 1 << " = " << flush;
        if (!(cin >> x1 >> y1 >> x2 >> y2)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x1, y1}) || !graph.PointAllowed({x2, y2})) {
            cerr << "Given ray/segment defining point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddInitialBounded({x1, y1}, {x2, y2}, Type::Segment);
    }

    for (int i = 0; i < initial[5]; ++i) {
        double a, b, r;
        cout << "\nGiven circle C" << i + 1 << ": enter center_x center_y radius.\n";
        cout << "  This represents (x-center_x)^2 + (y-center_y)^2 = radius^2.\n";
        cout << "C" << i + 1 << " = " << flush;
        if (!(cin >> a >> b >> r) || r < 0.0) {
            cerr << "Invalid circle. Radius must be non-negative.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        if (!graph.PointAllowed({a, b})) {
            cerr << "Given circle center is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddInitial(Element::FromCoefficients(a, b, Sq(r), Type::Circle));
    }
    graph.initialElementCount = graph.elements.size();

    cout << "\n[Step 5/6] Enter ALL construction goals simultaneously.\n";
    cout << "  Input three integers in this order: number_of_target_lines number_of_target_circles number_of_target_points\n";
    cout << "  Example: '2 1 3' means the final construction must contain 2 specified lines,\n";
    cout << "           1 specified circle, and 3 specified points at the same time.\n";
    cout << "Enter target counts L C P: " << flush;
    if (!ReadInteger(goalLines) || !ReadInteger(goalCircles) || !ReadInteger(goalPoints) ||
        goalLines < 0 || goalCircles < 0 || goalPoints < 0) {
        cerr << "Invalid target counts. All three counts must be non-negative integers.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    if (goalLines == 0 && goalCircles == 0 && goalPoints == 0) {
        cerr << "At least one target line, circle, or point is required.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    for (int i = 0; i < goalLines; ++i) {
        double a, b, c;
        cout << "\nTarget line GL" << i + 1 << ": enter a b c for a*x + b*y = c.\n";
        cout << "GL" << i + 1 << " (a b c) = " << flush;
        if (!(cin >> a >> b >> c)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        graph.goalElements.push_back(Element::FromCoefficients(a, b, c, Type::Line));
    }

    for (int i = 0; i < goalCircles; ++i) {
        double a, b, r;
        cout << "\nTarget circle GC" << i + 1 << ": enter center_x center_y radius.\n";
        cout << "GC" << i + 1 << " = " << flush;
        if (!(cin >> a >> b >> r) || r < 0.0) {
            cerr << "Invalid target circle. Radius must be non-negative.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        if (!graph.PointAllowed({a, b})) {
            cerr << "Target circle center is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.goalElements.push_back(Element::FromCoefficients(a, b, Sq(r), Type::Circle));
    }

    for (int i = 0; i < goalPoints; ++i) {
        double x, y;
        cout << "\nTarget point GP" << i + 1 << ": enter x y.\n";
        cout << "GP" << i + 1 << " = " << flush;
        if (!(cin >> x >> y)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x, y})) {
            cerr << "Target point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.goalPoints.push_back({x, y});
    }

    if (requestedSolutions == 0) {
        cout << "\n[Step 6/6] 需要几个不同的解？\n";
        cout << "请输入所需解数：" << flush;
        string countText;
        if (!(cin >> countText) || !ParseSolutionCount(countText, requestedSolutions)) {
            cerr << "Solution count must be an integer in 1..4294967295.\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }

    cout << "\n============================================================\n";
    cout << "Search configuration\n";
    cout << "  E limit: " << limit << "\n";
    cout << "  Requested distinct solutions: " << requestedSolutions << "\n";
    cout << "  Tools: " << (selectedMode == 3 ? "grid straightedge only (mode 3)" : toolType == 0 ? "compass only" : toolType == 1 ? "straightedge only" : "compass + straightedge") << "\n";
    if (graph.gridMode) {
        cout << "  Grid rectangle: [0," << graph.gridM << "] x [0," << graph.gridN << "] (closed)\n";
        cout << "  Automatic grid lines: " << graph.gridM + graph.gridN + 2 << " (free)\n";
        cout << "  Initial in-grid points: " << graph.points.size() << "\n";
    }
    cout << "  Goals: " << goalLines << " line(s), " << goalCircles << " circle(s), " << goalPoints << " point(s)\n";
    if (!certificatePath.empty() && filesystem::exists(certificatePath))
        throw invalid_argument("Certificate file already exists; choose a new path.");
    optional<Graph> certificateInitial;
    if (!certificatePath.empty()) certificateInitial = graph;
    heuristicOptions.threads = threads;
    cout << "  Search engine: " << (heuristic ? "v11 heuristic portfolio beam" : "v10 exhaustive DFS") << "\n";
    if (heuristic) {
        cout << "  Coverage thread slots: " << (heuristicOptions.coverageThreads<0 ? "auto" : to_string(heuristicOptions.coverageThreads))
             << " (shared-prefix DFS, not beam success)\n";
        cout << "  Beam width / branch limit per worker: " << heuristicOptions.beamWidth
             << " / " << heuristicOptions.branchLimit << "\n";
        cout << "  Seed: " << heuristicOptions.seed << "; restarts: " << heuristicOptions.restarts
             << " (0 = until time budget)\n";
        cout << "  Heuristic search may discard paths; failure is NOT exhaustion or an impossibility proof.\n";
    }
    const bool parallelCanSplit = !heuristic && threads > 1 && limit > 1;
    const size_t effectiveStreamDedupEntries = streamDedupEntries;
    constexpr size_t AUTO_FRONTIER_TASKS_PER_WORKER = 8;
    const size_t autoFrontierTarget = max<size_t>(
        static_cast<size_t>(threads) * AUTO_FRONTIER_TASKS_PER_WORKER, 8);
    const uint16_t maxSplitDepth = static_cast<uint16_t>(max(1, limit - 2));

    // Manual depth is exact. Auto mode probes successive depths and stops at
    // the first frontier with enough independent tasks. Each probe is capped at
    // autoFrontierTarget, so a huge frontier is sampled only up to the amount
    // actually useful for load balancing. Planning shares the same global time
    // budget as the real search and is additionally capped at 10% (max 3 s).
    uint16_t splitDepth = static_cast<uint16_t>(
        min<int>(requestedTaskDepth ? requestedTaskDepth : 1, maxSplitDepth));
    vector<pair<uint16_t, size_t>> frontierProbeCounts;
    vector<bool> frontierProbeThreshold;
    bool frontierProbeTimedOut = false;
    double frontierPlanningSeconds = 0.0;

    // Stable, cache-line-isolated storage outlives every solver and the sampler.
    // Slot 0 belongs to the main-thread producer (or the single-thread solver).
    const size_t progressSlotCount = heuristic ? threads : parallelCanSplit ? static_cast<size_t>(threads) + 1 : 1;
    auto progressSlots = make_unique<ProgressSlot[]>(progressSlotCount);
    ProgressState progressState(limit, heuristic ? threads : parallelCanSplit ? threads : 1, parallelCanSplit);
    progressState.heuristic = heuristic;
    const auto searchStartTime = chrono::steady_clock::now();
    const auto globalDeadline = searchStartTime + chrono::duration_cast<chrono::steady_clock::duration>(
        chrono::duration<double>(timeLimitSeconds));
    ProgressReporter progressReporter(progressState, progressSlots.get(), progressSlotCount,
        searchStartTime, timeLimitSeconds, progressInterval, progressEnabled);

    if (parallelCanSplit && requestedTaskDepth == 0) {
        const double planBudgetSeconds = min(3.0, max(0.05, timeLimitSeconds * 0.10));
        const auto planDeadline = min(globalDeadline, searchStartTime +
            chrono::duration_cast<chrono::steady_clock::duration>(
                chrono::duration<double>(planBudgetSeconds)));
        for (uint16_t d = 1; d <= maxSplitDepth; ++d) {
            Graph probeGraph = graph;
            Solver probe(toolType, symmetry, goalFirst, lowMemory,
                         effectiveStreamDedupEntries, 0, timeLimitSeconds);
            auto r = probe.ProbeFrontierTasks(
                probeGraph, limit, d, autoFrontierTarget, planDeadline);
            frontierProbeCounts.push_back({d, r.tasks});
            frontierProbeThreshold.push_back(r.thresholdReached);
            splitDepth = d;
            if (r.thresholdReached) break;
            if (r.timedOut || chrono::steady_clock::now() >= planDeadline) {
                frontierProbeTimedOut = true;
                break;
            }
        }
        frontierPlanningSeconds = chrono::duration<double>(
            chrono::steady_clock::now() - searchStartTime).count();
    }

    // Keep enough look-ahead to feed all workers without materializing a huge
    // fraction of a wide frontier. Manual and automatic depths use the same FIFO.
    const size_t queueFactor = 8;
    const size_t prefixQueueCapacity = parallelCanSplit
        ? max<size_t>(8, static_cast<size_t>(threads) * queueFactor) : 1;
    cout << "  Geometry EPS: " << scientific << setprecision(3) << EPS << defaultfloat << "\n";
    cout << "  Time limit: " << timeLimitSeconds << " seconds\n";
    cout << "  Worker threads: " << threads << " / " << availableThreads
         << " process-available (" << hardwareThreads << " hardware)\n";
    cout << "============================================================\n";
    cout << "Searching...\n" << flush;

    SearchStats stats;
    SolutionCollector solutions(requestedSolutions);
    bool found = false; // during search: requested quota reached

    bool timedOut = false;
    HeuristicResult heuristicResult;
    size_t totalTTBytes = 0;

    // Workers share only the bounded prefix FIFO, cancellation flags and
    // solution collector. Graphs and candidate tables stay private and are
    // reused across tasks. The main thread is the lightweight producer.
    const auto startTime = searchStartTime;
    progressState.searchStartOffset.store(chrono::duration<double>(
        chrono::steady_clock::now() - searchStartTime).count(), memory_order_relaxed);
    progressState.phase.store(ProgressPhase::Searching, memory_order_release);
    if (heuristic) {
        ParallelControl control;
        control.deadline = globalDeadline;
        heuristicResult = RunHeuristic(graph, limit, toolType, heuristicOptions,
            solutions, control, stats, progressEnabled ? progressSlots.get() : nullptr,
            progressEnabled ? progressSlotCount : 0);
        found = heuristicResult.quotaReached;
        timedOut = heuristicResult.timedOut;
    } else if (threads == 1 || limit <= 1) {
        Solver solver(toolType, symmetry, goalFirst, lowMemory,
                      effectiveStreamDedupEntries, ttMB * 1024ULL * 1024ULL,
                      timeLimitSeconds);
        solver.SetSolutionCollector(&solutions);
        solver.SetProgress(progressEnabled ? &progressSlots[0] : nullptr);
        found = solver.Search(graph, limit, stats);
        timedOut = solver.TimedOut();
        totalTTBytes = solver.TranspositionBytes();
    } else {
        ParallelControl control;
        control.deadline = globalDeadline;

        PrefixTaskQueue queue(control, prefixQueueCapacity);
        vector<SearchStats> workerStats(threads);
        vector<exception_ptr> workerErrors(threads);
        vector<thread> workers;
        workers.reserve(threads);
        exception_ptr producerError;
        SearchStats producerStats;
        bool producerFound = false;

        try {
            // Start consumers before producing the DFS frontier. Each Pop()
            // transfers exclusive ownership of a subtree. Graph rollback and
            // retained Solver tables avoid allocation churn at task boundaries.
            for (uint32_t wi = 0; wi < threads; ++wi) {
                workers.emplace_back([&, wi]() {
                    try {
                        PrefixTask task;
                        Graph localGraph = graph;
                        const Mark initialMark = localGraph.GetMark();
                        Solver localSolver(toolType, symmetry, goalFirst, lowMemory,
                                           effectiveStreamDedupEntries, 0, timeLimitSeconds);
                        localSolver.SetSolutionCollector(&solutions);
                        localSolver.SetProgress(progressEnabled ? &progressSlots[wi + 1] : nullptr);
                        while (queue.Pop(task)) {
                            localGraph.Rollback(initialMark);
                            SearchStats taskStats;
                            const bool localFound = localSolver.SearchPrefixTask(
                                localGraph, limit, task, taskStats, &control);
                            MergeSearchStats(workerStats[wi], taskStats);
                            if (!localFound && !control.stop.load(memory_order_acquire))
                                progressState.completedTasks.fetch_add(1, memory_order_relaxed);
                            if (localFound || control.stop.load(memory_order_acquire)) {
                                queue.Close();
                                break;
                            }
                        }
                    } catch (...) {
                        workerErrors[wi] = current_exception();
                        control.stop.store(true, memory_order_release);
                        queue.Close();
                    }
                });
            }

            Graph producerGraph = graph;
            Solver producer(toolType, symmetry, goalFirst, lowMemory,
                            effectiveStreamDedupEntries, 0, timeLimitSeconds);
            producer.SetSolutionCollector(&solutions);
            producer.SetProgress(progressEnabled ? &progressSlots[0] : nullptr);
            function<bool(const Graph&)> sink = [&](const Graph& g) {
                if (!queue.Push(g.elements, g.initialElementCount)) return false;
                progressState.producedTasks.fetch_add(1, memory_order_relaxed);
                return true;
            };
            producerFound = producer.ProduceFrontierTasks(
                producerGraph, limit, producerStats, &control, sink, splitDepth);
            if (producerFound) {
                control.found.store(true, memory_order_release);
                control.stop.store(true, memory_order_release);
            }
            // Only natural exhaustion makes the final task total knowable.
            if (!control.stop.load(memory_order_acquire) && !producer.TimedOut()) {
                progressState.producerDone.store(true, memory_order_release);
                progressState.phase.store(ProgressPhase::Draining, memory_order_release);
            }
        } catch (...) {
            producerError = current_exception();
            control.stop.store(true, memory_order_release);
        }
        // Normal producer exhaustion closes admission but lets workers drain.
        // A quota, timeout, or exception has already set stop, so it cancels.
        if (control.stop.load(memory_order_acquire))
            progressState.phase.store(ProgressPhase::Stopping, memory_order_release);
        queue.Close();
        for (thread& worker : workers) worker.join();

        // Never label an allocation/thread/worker failure as an exhausted search.
        if (!producerError) {
            for (const exception_ptr& error : workerErrors)
                if (error) { producerError = error; break; }
        }
        if (producerError) rethrow_exception(producerError);

        found = producerFound || control.found.load(memory_order_acquire);
        timedOut = control.timedOut.load(memory_order_acquire) && !found;
        MergeSearchStats(stats, producerStats);
        for (uint32_t wi = 0; wi < threads; ++wi)
            MergeSearchStats(stats, workerStats[wi]);
        totalTTBytes = 0;
    }
    const double seconds = chrono::duration<double>(chrono::steady_clock::now() - startTime).count();
    // Wake immediately even with a very long interval, before formatting results
    // or any optional interactive pause. The destructor also joins on exceptions.
    progressReporter.Finish();

    // Only now, after joins, read and format the stored results.
    const size_t resultCount = solutions.Count();
    const bool quotaReached = resultCount >= requestedSolutions;
    found = resultCount != 0;
    if (quotaReached) timedOut = false;
    if (found && certificateInitial) {
        ostringstream certificate;
        certificate << "{\"version\":\"v11\",\"eps\":" << setprecision(17) << EPS << ",\"solutions\":[";
        for (size_t i = 0; i < resultCount; ++i) {
            if (i) certificate << ',';
            WriteConstructionCertificate(certificate, *certificateInitial, solutions.Entries()[i].graph);
        }
        certificate << "]}\n";
        if (filesystem::exists(certificatePath)) throw runtime_error("Certificate file appeared during search; refusing overwrite.");
        ofstream output(certificatePath, ios::out | ios::binary);
        if (!output || !(output << certificate.str())) throw runtime_error("Cannot write construction certificate.");
    }
    cout << "\n============================================================\n";
    double solutionOutputSeconds = 0.0;
    if (found) {
        const auto outputStart = chrono::steady_clock::now();
        for (size_t i = 0; i < resultCount; ++i) {
            const Graph& solved = solutions.Entries()[i].graph;
            cout << "\n========== 第 " << i + 1 << " / " << resultCount
                 << " 个不同解（" << solved.elements.size() - solved.initialElementCount
                 << "E） ==========\n";
            if (readableOutput) readable_report::Print(solved, givenPointCount);
            else solved.PrintSolution();
        }
        cout.flush();
        solutionOutputSeconds = chrono::duration<double>(
            chrono::steady_clock::now() - outputStart).count();
    }
    if (quotaReached) {
        cout << "\n已返回所需的 " << requestedSolutions << " 个不同解。\n";
        cout << "Result status: QUOTA_REACHED\n";
    } else if (timedOut) {
        cout << "\nSEARCH STOPPED: time limit reached (" << timeLimitSeconds << " seconds).\n";
        cout << "已找到并输出 " << resultCount << " / " << requestedSolutions
             << " 个不同解\n";
        cout << "Result status: TIMEOUT_PARTIAL\n";
    } else if (heuristic) {
        cout << "Heuristic search stopped before the requested quota. Unvisited paths remain possible.\n";
        cout << "Result status: HEURISTIC_STOPPED\n";
    } else {
        if (!found) cout << "No solution found within the specified E limit.\n";
        cout << "The search completed without timing out.\n";
        cout << "Result status: EXHAUSTED\n";
    }
    cout << "Requested solutions:" << requestedSolutions << "\n";
    cout << "Distinct solutions:" << resultCount << "\n";
    cout << "Successful state visits:" << solutions.SuccessfulVisits() << "\n";
    cout << fixed << setprecision(6);
    cout << "Search Time:" << seconds << "\n";
    if (found) cout << "Solution output time:" << setprecision(9)
                    << solutionOutputSeconds << setprecision(6) << "\n";
    cout << "Worker threads:" << threads << "\n";
    cout << "Nodes:" << stats.nodes << "\n";
    cout << "Raw candidates:" << stats.rawCandidates << "\n";
    cout << "Unique candidates:" << stats.uniqueCandidates << "\n";
    if (heuristic) {
        const auto& m = heuristicResult.metrics;
        cout << "Heuristic restarts:" << m.restarts << "\n";
        cout << "Heuristic layers:" << m.layers << "\n";
        cout << "Expanded beam states:" << m.expanded << "\n";
        cout << "Evaluated successors:" << m.evaluated << "\n";
        cout << "Candidate paths discarded:" << m.candidateDiscarded << "\n";
        cout << "Beam paths discarded:" << m.beamDiscarded << "\n";
        cout << "Tail helper calls:" << m.tailCalls << "\n";
        cout << "Beam successful state visits:" << m.beamSolutions << "\n";
        cout << "Helper successful state visits:" << m.helperSolutions << "\n";
        cout << "Family-diversity discards:" << m.familyMerged << "\n";
        cout << "Rendezvous proposals:" << m.rendezvousProposals << "\n";
        cout << "Rendezvous replays:" << m.rendezvousReplays << "\n";
        cout << "Rendezvous successful state visits:" << m.rendezvousSolutions << "\n";
        cout << "Chain join calls:" << m.chainCalls << "\n";
        cout << "Chain join proposals:" << m.chainProposals << "\n";
        cout << "Chain join replays:" << m.chainReplays << "\n";
        cout << "Chain join successful state visits:" << m.chainSolutions << "\n";
        cout << "Point join calls:" << m.pointJoinCalls << "\n";
        cout << "Point join proposals:" << m.pointJoinProposals << "\n";
        cout << "Point join replays:" << m.pointJoinReplays << "\n";
        cout << "Point join successful state visits:" << m.pointJoinSolutions << "\n";
        cout << "Coverage tasks:" << m.coverageTasks << "\n";
        cout << "Coverage successful state visits:" << m.coverageSolutions << "\n";
        cout << "Equal-radius probe calls:" << m.radiusProbeCalls << "\n";
        cout << "Equal-radius probe prefixes:" << m.radiusProbePrefixes << "\n";
        cout << "Equal-radius probe completions:" << m.radiusProbeCompletions << "\n";
        cout << "Equal-radius probe successful state visits:" << m.radiusProbeSolutions << "\n";
        cout << "Prerequisite probe calls:" << m.prerequisiteCalls << "\n";
        cout << "Prerequisite operations:" << m.prerequisiteApplied << "\n";
        cout << "Goal finish calls:" << m.goalFinishCalls << "\n";
        cout << "Prerequisite successful state visits:" << m.prerequisiteSolutions << "\n";
        cout << "Peak beam per worker:" << m.peakBeam << "\n";
        cout << "Search completeness: HEURISTIC (no exhaustive guarantee)\n";
    }
    cout << "============================================================\n";

    int exitCode = 2;
    if (timedOut) exitCode = 3; // also for a partial list: it is not exhausted
    else if (quotaReached) exitCode = 0;
    else if (heuristic) exitCode = 4;
    else if (found) exitCode = 0;

    return ExitWithPause(exitCode, pauseOnExit);
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    ios::sync_with_stdio(false);
    cin.tie(nullptr);
    // The sampler owns stderr while searching. Its output must not implicitly
    // flush cout from another thread while the main thread prints configuration.
    cerr.tie(nullptr);
    bool pauseOnExit = false;
    try {
        return Run(argc, argv, pauseOnExit);
    } catch (const bad_alloc&) {
        cerr << "Memory allocation failed; search was not exhausted.\n";
    } catch (const invalid_argument& ex) {
        cerr << "Invalid input/option: " << ex.what() << '\n';
    } catch (const exception& ex) {
        cerr << "Execution failed; search was not exhausted: " << ex.what() << '\n';
    } catch (...) {
        cerr << "Execution failed; search was not exhausted: unknown exception\n";
    }
    return ExitWithPause(1, pauseOnExit);
}
