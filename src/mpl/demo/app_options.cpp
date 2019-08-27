#include <mpl/demo/app_options.hpp>
#include <getopt.h>
#include <iostream>

float mpl::demo::OptionParser<float>::parse(
    const std::string& name, const char *arg, char **endp)
{
    if (!*arg)
        throw std::invalid_argument(name + " is required");
    
    float v = std::strtof(arg, endp);
    if (*endp == arg)
        throw std::invalid_argument("bad value for " + name + ": " + arg);
    return v;
}

double mpl::demo::OptionParser<double>::parse(
    const std::string& name, const char *arg, char **endp)
{
    if (!*arg)
        throw std::invalid_argument(name + " is required");
    
    float v = std::strtod(arg, endp);
    if (*endp == arg)
        throw std::invalid_argument("bad value for " + name + ": " + arg);
    return v;
}

void mpl::demo::AppOptions::usage(const char *argv0) {
    std::cerr << "Usage: " << argv0 << R"( [options]
Options:
  -a, --algorithm=(rrt|cforest)
  -c, --coordinator=HOST:PORT
  -I, --problem-id=ID
  -t, --time-limit=TIME
  -e, --env=MESH
  -r, --robot=MESH
  -s, --start=W,I,J,K,X,Y,Z
  -g, --goal=W,I,J,K,X,Y,Z
  -m, --min=X,Y,Z
  -M, --max=X,Y,Z
  -d, --discretization=DIST   (0 means use default)
)";
}

mpl::demo::AppOptions::AppOptions(int argc, char *argv[]) {
    static struct option longopts[] = {
        { "algorithm", required_argument, NULL, 'a' },
        { "env", required_argument, NULL, 'e' },
        { "robot", required_argument, NULL, 'r' },
        { "goal", required_argument, NULL, 'g' },
        { "start", required_argument, NULL, 's' },
        { "min", required_argument, NULL, 'm' },
        { "max", required_argument, NULL, 'M' },
        { "coordinator", required_argument, NULL, 'c' },
        { "problem-id", required_argument, NULL, 'I' },
        { "time-limit", required_argument, NULL, 't' },
        { "discretization", required_argument, NULL, 'd' },
                
        { NULL, 0, NULL, 0 }
    };

    for (int ch ; (ch = getopt_long(argc, argv, "a:e:r:g:s:m:M:c:I:t:d:", longopts, NULL)) != -1 ; ) {
        char *endp;
                
        switch (ch) {
        case 'a':
            algorithm_ = optarg;
            break;
        case 'e':
            env_ = optarg;
            break;
        case 'r':
            robot_ = optarg;
            break;
        case 'g':
            goal_ = optarg;
            break;
        case 's':
            start_ = optarg;
            break;
        case 'm':
            min_ = optarg;
            break;
        case 'M':
            max_ = optarg;
            break;
        case 'c':
            coordinator_ = optarg;
            break;
        case 'I':
            problemId_ = strtoull(optarg, &endp, 0);
            if (endp == optarg || *endp)
                throw std::invalid_argument("bad value for --problem-id");
            break;
        case 't':
            timeLimit_ = std::strtod(optarg, &endp);
            if (endp == optarg || *endp || timeLimit_ < 0)
                throw std::invalid_argument("bad value for --time-limit");
            break;
        case 'd':
            discretization_ = std::strtod(optarg, &endp);
            if (endp == optarg || *endp || discretization_ < 0)
                throw std::invalid_argument("bad value for --discretization");
            break;
        default:
            usage(argv[0]);
            throw std::invalid_argument("unknown option");
        }            
    }
    
}
