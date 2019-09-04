#include <mpl/demo/lambda_common.hpp>
// main.cpp
#include <aws/lambda-runtime/runtime.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <iostream>
#include <memory>
#include <string>
#include <iostream>

using namespace aws::lambda_runtime;

invocation_response my_handler(invocation_request const& request) {
    try
    {
    using namespace Aws::Utils::Json;
//    JsonValue json(request.payload);
    Aws::String payload(request.payload.c_str(), request.payload.size());
    JsonValue json(payload);

    if (!json.WasParseSuccessful()) {
        return invocation_response::failure("Failed to parse input JSON", "InvalidJSON");
    }

    auto v = json.View();

    if (!v.ValueExists("scenario") || !v.ValueExists("coordinator") || !v.ValueExists("start") ||
        !v.ValueExists("goal") || !v.ValueExists("min") || !v.ValueExists("max") || !v.ValueExists("algorithm") ||
        !v.ValueExists("env") || !v.ValueExists("robot") || !v.ValueExists("envFrame")) {
        return invocation_response::failure("Missing input value.", "InvalidJSON");
    }

    Aws::String aws_s;
    aws_s = v.GetString("scenario");
    std::string scenario(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("coordinator");
    std::string coordinator(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("start");
    std::string start(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("goal");
    std::string goal(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("min");
    std::string min(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("max");
    std::string max(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("algorithm");
    std::string algorithm(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("env");
    std::string env(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("robot");
    std::string robot(aws_s.c_str(), aws_s.size());
    aws_s = v.GetString("envFrame");
    std::string envFrame(aws_s.c_str(), aws_s.size());

    mpl::demo::AppOptions options;
    options.scenario_ = scenario;
    options.coordinator_ = coordinator;
    options.start_ = start;
    options.goal_ = goal;
    options.min_ = min;
    options.max_ = max;
    options.algorithm_ = algorithm;
    options.env_ = env;
    options.robot_ = robot;
    options.envFrame_ = envFrame;

    std::uint64_t tester = 123;
    options.problemId_ = tester;

    mpl::demo::runSelectPlanner(options);
    return invocation_response::success(start, "application/json");
    } catch (const std::invalid_argument& ex) {
        std::cerr << "Invalid argument: " << ex.what() << std::endl;
        return invocation_response::failure(ex.what(), "InvalidArgument");
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return invocation_response::failure(ex.what(), "Exception");
    }
}


int main()
{
   run_handler(my_handler);
   return 0;
}

