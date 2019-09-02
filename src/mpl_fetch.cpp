#include <mpl/demo/fetch_robot.hpp>
#include <iostream>

int main(int argc, char *argv[]) {
    using S = double;
    using namespace mpl::demo;
    using Robot = FetchRobot<S>;

    std::mt19937_64 rng;

    typename Robot::Frame envFrame;
    envFrame.setIdentity();
    envFrame.translation() << 0.57, -0.90, 0;
    Eigen::AngleAxis<S> envAngle(-1.570796326794897, Eigen::Matrix<S,3,1>::UnitZ());
    envFrame.linear() = envAngle.toRotationMatrix();
    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",");
    
    BlenderPy bpy(std::cout);
    bpy << "#!/usr/bin/env blender --python";
    bpy << "import bpy";
    bpy << "bpy.ops.object.delete(use_global=False)";

    bpy << "Env = bpy.data.objects.new(\"Env\", None)";
    bpy << "bpy.context.collection.objects.link(Env)";
    bpy << "bpy.ops.wm.collada_import(filepath=\"../../resources/AUTOLAB.dae\")";
    bpy << "for obj in bpy.context.selected_objects:";
    bpy << "    obj.constraints.new(type='CHILD_OF')";
    bpy << "    obj.constraints['Child Of'].target = Env";
    bpy << "Env.location = (" << envFrame.translation().format(fmt) << ")";
    bpy << "Env.rotation_mode = 'AXIS_ANGLE'";
    bpy << "Env.rotation_axis_angle = (" << envAngle.angle() << ',' << envAngle.axis().format(fmt) << ")";
    
    Robot robot;
    robot.toArticulatedBlenderScript(bpy, "../../resources/fetch/");

    robot.setConfig(Robot::Config::Zero()); // restConfig());
    assert(!robot.selfCollision());


    // envFrame = envFrame.inverse();

    // should be around 0.82, 0.53, 0.88
    typename Robot::Frame ikTarget;
    ikTarget.setIdentity();
    ikTarget.translation() << -1.07,0.16,0.88;

    // std::clog << "ikTarget\n" << ikTarget.matrix() << "\n"
    //           << "env * ikTarget\n"    << (envFrame*ikTarget).matrix() << "\n"
    //           << "env^-1 * ikTarget\n" << (envFrame.inverse()*ikTarget).matrix() << "\n"
    //           << "ikTarget * env\n"    << (ikTarget * envFrame).matrix() << "\n"
    //           << "ikTarget * env^-1\n" << (ikTarget * envFrame.inverse()).matrix() << "\n";
    
    ikTarget = envFrame * ikTarget;
    // ikTarget.translation() -= envFrame.translation();
    
    std::clog << "IK Target:\n" << ikTarget.matrix() << std::endl;

    std::clog << "Gripper Axis:\n" << robot.gripperAxis().matrix() << std::endl;
    
    Eigen::Matrix<S, 6, 1> L;
    // L << 1, 1, 1,   1, 1, 0.00001; // /1.570796326794897;

    L << 0.01, 0.01, 0.01, 0.01, 0.01, 1.570796326794897;
    S eps = L.minCoeff();
    L = eps / L.array();

    for (int iter = 0 ; iter<100 ; ++iter) {
        robot.setConfig(Robot::randomConfig(rng));
        if (robot.ik(ikTarget, L, eps)) {
            std::clog << "IK SOLVED after " << iter << std::endl;
            break;
        }
    }
    
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
