#include <mpl/demo/lambda_common.hpp>
#include <iostream>

int main(int argc, char *argv[]) try {
    static const std::string resourceDirectory = "../../resources/";
    mpl::demo::AppOptions options(argc, argv);

    if (!options.env_.empty())
        options.env_ = resourceDirectory + options.env_;
    if (!options.robot_.empty())
        options.robot_ = resourceDirectory + options.robot_;

    mpl::demo::runSelectPlanner(options);
    return EXIT_SUCCESS;
} catch (const std::invalid_argument& ex) {
    std::cerr << "Invalid argument: " << ex.what() << std::endl;
    return EXIT_FAILURE;
} catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << std::endl;
    return EXIT_FAILURE;
}
