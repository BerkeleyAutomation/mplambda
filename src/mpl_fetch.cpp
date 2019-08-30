#include <mpl/demo/fetch_robot.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
    using S = double;
    using namespace mpl::demo;
    using Robot = FetchRobot<S>;

    std::mt19937_64 rng;

    BlenderPy bpy(std::cout);
    bpy << "#!/usr/bin/env blender --python";
    bpy << "import bpy";
    bpy << "import os.path";
    bpy << "bpy.ops.object.delete(use_global=False)";

    Robot robot;
    robot.toArticulatedBlenderScript(bpy, "../../resources/fetch/");

    for (int frame=0 ; frame<100 ; ++frame) {
        do {
            robot.setConfig(Robot::randomConfig(rng));
        } while (robot.selfCollision());

        robot.updateArticulatedBlenderScript(bpy);
        robot.keyframeInsert(bpy, frame*20 + 1);
    }
    
    return EXIT_SUCCESS;
}
