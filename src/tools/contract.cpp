#include "contractor/contractor.hpp"
#include "contractor/contractor_config.hpp"
#include "util/exception.hpp"
#include "util/simple_logger.hpp"
#include "util/version.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/errors.hpp>

#include <tbb/task_scheduler_init.h>

#include <cstdlib>
#include <exception>
#include <new>
#include <ostream>

using namespace osrm;

#include <execinfo.h>
#include <errno.h>
#include <cxxabi.h>
#include <stdio.h>
#include <signal.h>

static inline void printStackTrace(FILE *out = stderr, unsigned int max_frames = 63)
{
    fprintf(out, "stack trace:\n");

    // storage array for stack trace address data
    void *addrlist[max_frames + 1];

    // retrieve current stack addresses
    unsigned int addrlen = backtrace(addrlist, sizeof(addrlist) / sizeof(void *));

    if (addrlen == 0)
    {
        fprintf(out, "  \n");
        return;
    }

    // resolve addresses into strings containing "filename(function+address)",
    // Actually it will be ## program address function + offset
    // this array must be free()-ed
    char **symbollist = backtrace_symbols(addrlist, addrlen);

    size_t funcnamesize = 1024;
    char funcname[1024];

    // iterate over the returned symbol lines. skip the first, it is the
    // address of this function.
    for (unsigned int i = 4; i < addrlen; i++)
    {
        char *begin_name = NULL;
        char *begin_offset = NULL;
        char *end_offset = NULL;

// find parentheses and +address offset surrounding the mangled name
#ifdef DARWIN
        // OSX style stack trace
        for (char *p = symbollist[i]; *p; ++p)
        {
            if ((*p == '_') && (*(p - 1) == ' '))
                begin_name = p - 1;
            else if (*p == '+')
                begin_offset = p - 1;
        }

        if (begin_name && begin_offset && (begin_name < begin_offset))
        {
            *begin_name++ = '\0';
            *begin_offset++ = '\0';

            // mangled name is now in [begin_name, begin_offset) and caller
            // offset in [begin_offset, end_offset). now apply
            // __cxa_demangle():
            int status;
            char *ret = abi::__cxa_demangle(begin_name, &funcname[0], &funcnamesize, &status);
            if (status == 0)
            {
                funcname = ret; // use possibly realloc()-ed string
                fprintf(out, "  %-30s %-40s %s\n", symbollist[i], funcname, begin_offset);
            }
            else
            {
                // demangling failed. Output function name as a C function with
                // no arguments.
                fprintf(out, "  %-30s %-38s() %s\n", symbollist[i], begin_name, begin_offset);
            }

#else  // !DARWIN - but is posix
        // not OSX style
        // ./module(function+0x15c) [0x8048a6d]
        for (char *p = symbollist[i]; *p; ++p)
        {
            if (*p == '(')
                begin_name = p;
            else if (*p == '+')
                begin_offset = p;
            else if (*p == ')' && (begin_offset || begin_name))
                end_offset = p;
        }

        if (begin_name && end_offset && (begin_name < end_offset))
        {
            *begin_name++ = '\0';
            *end_offset++ = '\0';
            if (begin_offset)
                *begin_offset++ = '\0';

            // mangled name is now in [begin_name, begin_offset) and caller
            // offset in [begin_offset, end_offset). now apply
            // __cxa_demangle():

            int status = 0;
            char *ret = abi::__cxa_demangle(begin_name, funcname, &funcnamesize, &status);
            char *fname = begin_name;
            if (status == 0)
                fname = ret;

            if (begin_offset)
            {
                fprintf(out,
                        "  %-30s ( %-40s  + %-6s) %s\n",
                        symbollist[i],
                        fname,
                        begin_offset,
                        end_offset);
            }
            else
            {
                fprintf(out, "  %-30s ( %-40s    %-6s) %s\n", symbollist[i], fname, "", end_offset);
            }
#endif // !DARWIN - but is posix
        }
        else
        {
            // couldn't parse the line? print the whole line.
            fprintf(out, "  %-40s\n", symbollist[i]);
        }
    }

    free(symbollist);
}

void abortHandler(int signum, siginfo_t *si, void *unused)
{
    // associate each signal with a signal name string.
    const char *name = NULL;
    switch (signum)
    {
    case SIGABRT:
        name = "SIGABRT";
        break;
    case SIGSEGV:
        name = "SIGSEGV";
        break;
    case SIGBUS:
        name = "SIGBUS";
        break;
    case SIGILL:
        name = "SIGILL";
        break;
    case SIGFPE:
        name = "SIGFPE";
        break;
    }

    // Notify the user which signal was caught. We use printf, because this is the
    // most basic output function. Once you get a crash, it is possible that more
    // complex output systems like streams and the like may be corrupted. So we
    // make the most basic call possible to the lowest level, most
    // standard print function.
    if (name)
        fprintf(stderr, "Caught signal %d (%s)\n", signum, name);
    else
        fprintf(stderr, "Caught signal %d\n", signum);

    // Dump a stack trace.
    // This is the function we will be implementing next.
    printStackTrace();

    // If you caught one of the above signals, it is likely you just
    // want to quit your program right now.
    exit(signum);
}

enum class return_code : unsigned
{
    ok,
    fail,
    exit
};

return_code parseArguments(int argc, char *argv[], contractor::ContractorConfig &contractor_config)
{
    // declare a group of options that will be allowed only on command line
    boost::program_options::options_description generic_options("Options");
    generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message");

    // declare a group of options that will be allowed on command line
    boost::program_options::options_description config_options("Configuration");
    config_options.add_options()(
        "threads,t",
        boost::program_options::value<unsigned int>(&contractor_config.requested_num_threads)
            ->default_value(tbb::task_scheduler_init::default_num_threads()),
        "Number of threads to use")(
        "core,k",
        boost::program_options::value<double>(&contractor_config.core_factor)->default_value(1.0),
        "Percentage of the graph (in vertices) to contract [0..1]")(
        "segment-speed-file",
        boost::program_options::value<std::vector<std::string>>(
            &contractor_config.segment_speed_lookup_paths)
            ->composing(),
        "Lookup files containing nodeA, nodeB, speed data to adjust edge weights")(
        "turn-penalty-file",
        boost::program_options::value<std::vector<std::string>>(
            &contractor_config.turn_penalty_lookup_paths)
            ->composing(),
        "Lookup files containing from_, to_, via_nodes, and turn penalties to adjust turn weights")(
        "level-cache,o",
        boost::program_options::value<bool>(&contractor_config.use_cached_priority)
            ->default_value(false),
        "Use .level file to retain the contaction level for each node from the last run.")(
        "edge-weight-updates-over-factor",
        boost::program_options::value<double>(&contractor_config.log_edge_updates_factor)
            ->default_value(0.0),
        "Use with `--segment-speed-file`. Provide an `x` factor, by which Extractor will log edge "
        "weights updated by more than this factor");

    // hidden options, will be allowed on command line, but will not be shown to the user
    boost::program_options::options_description hidden_options("Hidden options");
    hidden_options.add_options()(
        "input,i",
        boost::program_options::value<boost::filesystem::path>(&contractor_config.osrm_input_path),
        "Input file in .osm, .osm.bz2 or .osm.pbf format");

    // positional option
    boost::program_options::positional_options_description positional_options;
    positional_options.add("input", 1);

    // combine above options for parsing
    boost::program_options::options_description cmdline_options;
    cmdline_options.add(generic_options).add(config_options).add(hidden_options);

    const auto *executable = argv[0];
    boost::program_options::options_description visible_options(
        "Usage: " + boost::filesystem::path(executable).filename().string() +
        " <input.osrm> [options]");
    visible_options.add(generic_options).add(config_options);

    // parse command line options
    boost::program_options::variables_map option_variables;
    try
    {
        boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
                                          .options(cmdline_options)
                                          .positional(positional_options)
                                          .run(),
                                      option_variables);
    }
    catch (const boost::program_options::error &e)
    {
        util::SimpleLogger().Write(logERROR) << e.what();
        return return_code::fail;
    }

    if (option_variables.count("version"))
    {
        util::SimpleLogger().Write() << OSRM_VERSION;
        return return_code::exit;
    }

    if (option_variables.count("help"))
    {
        util::SimpleLogger().Write() << visible_options;
        return return_code::exit;
    }

    boost::program_options::notify(option_variables);

    if (!option_variables.count("input"))
    {
        util::SimpleLogger().Write() << visible_options;
        return return_code::fail;
    }

    return return_code::ok;
}

int main(int argc, char *argv[]) try
{

    struct sigaction sa;
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = abortHandler;
    sigemptyset(&sa.sa_mask);

    sigaction(SIGABRT, &sa, NULL);
    sigaction(SIGSEGV, &sa, NULL);
    sigaction(SIGBUS, &sa, NULL);
    sigaction(SIGILL, &sa, NULL);
    sigaction(SIGFPE, &sa, NULL);
    sigaction(SIGPIPE, &sa, NULL);

    util::LogPolicy::GetInstance().Unmute();
    contractor::ContractorConfig contractor_config;

    const return_code result = parseArguments(argc, argv, contractor_config);

    if (return_code::fail == result)
    {
        return EXIT_FAILURE;
    }

    if (return_code::exit == result)
    {
        return EXIT_SUCCESS;
    }

    contractor_config.UseDefaultOutputNames();

    if (1 > contractor_config.requested_num_threads)
    {
        util::SimpleLogger().Write(logERROR) << "Number of threads must be 1 or larger";
        return EXIT_FAILURE;
    }

    const unsigned recommended_num_threads = tbb::task_scheduler_init::default_num_threads();

    if (recommended_num_threads != contractor_config.requested_num_threads)
    {
        util::SimpleLogger().Write(logWARNING)
            << "The recommended number of threads is " << recommended_num_threads
            << "! This setting may have performance side-effects.";
    }

    if (!boost::filesystem::is_regular_file(contractor_config.osrm_input_path))
    {
        util::SimpleLogger().Write(logERROR)
            << "Input file " << contractor_config.osrm_input_path.string() << " not found!";
        return EXIT_FAILURE;
    }

    util::SimpleLogger().Write() << "Input file: "
                                 << contractor_config.osrm_input_path.filename().string();
    util::SimpleLogger().Write() << "Threads: " << contractor_config.requested_num_threads;

    tbb::task_scheduler_init init(contractor_config.requested_num_threads);

    return contractor::Contractor(contractor_config).Run();
}
catch (const std::bad_alloc &e)
{
    util::SimpleLogger().Write(logERROR) << "[exception] " << e.what();
    util::SimpleLogger().Write(logERROR)
        << "Please provide more memory or consider using a larger swapfile";
    return EXIT_FAILURE;
}