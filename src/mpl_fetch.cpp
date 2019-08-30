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
    bpy << "bpy.ops.object.delete(use_global=False)";

    // bpy << "environment = bpy.data.objects.new(\"environment\", None)";
    // bpy << "bpy.ops.wm.collada_import(filepath=\"../../resources/AUTOLAB.dae\")";
    // bpy << "for obj in bpy.context.selected_objects:";
    // bpy << "    obj.constraints.new(type='CHILD_OF')";
    // bpy << "    obj.constraints['Child Of'].target = environment";
    // bpy << "environment.location = (0.237, -0.669, 0)";
    // bpy << "environment.rotation_mode = 'AXIS_ANGLE'";
    // bpy << "environment.rotation_axis_angle = (3.141592654,0,0,1)";
    
    Robot robot;
    robot.toArticulatedBlenderScript(bpy, "../../resources/fetch/");

    robot.setConfig(Robot::restConfig());
    assert(!robot.selfCollision());
    
    robot.updateArticulatedBlenderScript(bpy);

    
    // for (int frame=0 ; frame<1 ; ++frame) {
    //     do {
    //         robot.setConfig(Robot::randomConfig(rng));
    //     } while (robot.selfCollision());

    //     robot.updateArticulatedBlenderScript(bpy);
    //     robot.keyframeInsert(bpy, frame*20 + 1);
    // }
    
    return EXIT_SUCCESS;
}
