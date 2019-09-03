#include <mpl/demo/fetch_robot.hpp>
#include <iostream>


template <class T, std::size_t I>
T makeState() {
}

template <class T>
struct Make;

template <class S, int rows>
struct Make<Eigen::Matrix<S, rows, 1>> {
    using T = Eigen::Matrix<S, rows, 1>;

    template <std::size_t I, class F, class ... R>
    static void build(T& value, F first, R ... rest) {
        value[I] = first;
        if constexpr (sizeof...(R) > 0)
            build<I+1>(value, rest...);
    }
    
    template <class ... Args>
    static T apply(Args ... args) {
        T value;
        build<0>(value, args...);
        return value;
    }
};

template <class T, class ... S>
void addPath(std::vector<T>& path, S ... args) {
    path.emplace_back(Make<T>::apply(args...));
}

template <class T>
std::vector<T> makePath001() {
    std::vector<T> P;
    addPath(P, 0.38615, 0.508560150174262, 0.375553424018286, -5.94062382778246, 0.214223003680049, 1.29160901715679, -1.72532145407213, -0.987251885861771);
    addPath(P, 0.386142088805617, 0.457014231831145, 0.746881974502196, -3.27722187130138, -0.109209667430568, 1.90544281509359, -1.80977097230234, 0.612925025920071);
    addPath(P, 0.386147010371738, 0.213112583815886, 0.962163198239821, -1.77671577169881, 0.259088727928865, 1.38685795832772, -1.8456573832558, 0.958631445454626);
    addPath(P, 0.386145003700433, 0.614387508495193, 1.38501344006415, -1.37234068808198, 1.73561998803765, 0.765197681841562, -0.391566965391333, 0.268679416206135);
    addPath(P, 0.386142683541026, 1.3021379471638, 0.939420679367169, -1.53199806647382, 2.03464732206441, 0.410895605036528, -0.0902114766771591, 0.628458937297813);
    addPath(P, 0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0);
    return P;
}

template <class T>
std::vector<T> makePath002() {
    std::vector<T> P;
    addPath(P, 0.38615, 0.801973559455454, 0.414672969328816, 8.16937089205058, -0.606338545452261, 2.91416009655183, 1.19764730707252, 8.41138287434359);
    addPath(P, 0.385897670255518, 0.606077537713378, -0.884090365131937, 6.83217435720473, 0.460270739711006, 1.58860436519146, 1.50392138179204, 7.23765015790164);
    addPath(P, 0.385351817228527, 0.580176091443043, -0.671758754343179, 5.6392728840885, 0.727143258495745, 2.7607723104927, 0.27379945492407, 7.07591190529792);
    addPath(P, 0.380617217689278, 0.434382757123331, -0.151758754343179, 5.3771385914189, 0.352363106849085, 2.39070294374439, 0.603581733939425, 6.8634356478348);
    addPath(P, 0.376185526244643, 0.954382757123331, 0.0969406257579499, 5.03725859880117, -0.162308844441939, 2.00008807823459, 0.630739457442079, 7.12387573008954);
    addPath(P, 0.379870289931012, 0.509945475196034, -0.475042625622348, 3.58685790994216, 0.357780202760977, 1.06318266383589, 0.114201105547848, 5.64915725026278);
    addPath(P, 0.328705751927573, 0.356978412865531, 0.430438764345975, 2.18133426106463, 1.3863671645541, -0.570345858298713, 1.18333133751567, 5.92028963442377);
    addPath(P, 0.301035829454967, 0.420053448466591, 0.23956977580043, 1.89641399726345, 1.06841651361516, -1.09034585829871, 0.747685775699538, 5.70642283235834);
    addPath(P, 0.285227417187641, 0.40507466036898, 0.531369741155522, 0.308098724319502, 1.26045159225192, -0.710792138971174, 1.41116869044652, 4.01578091281487);
    addPath(P, 0.20932923954041, 0.92507466036898, 1.00988566539978, -0.16765833519867, 1.35272063886886, -1.12978450075218, 0.956893544427132, 3.60041342520087);
    addPath(P, 0.195092773862392, 0.957732635601546, 1.02551575501173, -0.332437156969361, 1.29615433598662, -0.609784500752176, 1.08226601953315, 3.12);
    addPath(P, 0.181972674187058, 1.0013256671465, 1.12246167185081, -0.738775518195535, 1.23517282026416, -0.290686451171191, 1.10593880170086, 2.6);
    addPath(P, 0.169935609665969, 1.15206888532727, 1.17249512110764, -0.635522101685855, 1.3480506226988, 0.134795490335083, 1.29195599108338, 2.08);
    addPath(P, 0.154848162398206, 1.31276650293488, 1.28617010793053, -0.549573104627645, 1.40164278821011, 0.58536938378269, 1.42983149844494, 1.56);
    addPath(P, 0.137738537813743, 1.34629610653129, 1.38310915198056, -0.239688476500695, 1.53854534703539, 0.361137629444734, 1.41570658490416, 1.04);
    addPath(P, 0.126078487360453, 1.45769746988383, 1.38586692776312, -0.420527712393827, 1.54619036943838, 0.275651385130523, 1.5195905562347, 0.52);
    addPath(P, 0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0);
    return P;
}

template <class T>
std::vector<T> makePath003() {
    std::vector<T> P;
    addPath(P, 0.342009765026439, 0.603033629069347, 0.818687809758623, 3.31057461898144, 0.848715586415808, -4.82148658189959, 1.47040524682231, 1.54549869395757);
    addPath(P, 0.13951225411896, 0.177372211371964, 0.669570517828946, 3.16871622633316, 2.01004541446168, -3.55499255270217, 1.20856988888469, 1.83257462777608);
    addPath(P, 0.118429956522941, -0.342627788628036, 0.170802693769544, 3.17717350755562, 1.87710563661761, -3.16746335061948, 1.03917742161299, 2.14931137502374);
    addPath(P, 0.313885665453301, 0.562922266929196, -0.145252703743405, 2.4988729261165, 1.40382031551758, -2.94597753894417, 0.669219822218754, 2.50281901111157);
    addPath(P, 0.24639855138333, 1.13795487970897, 0.424511833042091, 1.86994323893161, 0.893667890715115, -2.26734423959981, 0.915675326829185, 2.28335323526018);
    addPath(P, 0.127289748562299, 0.997175518651635, 0.474512149390198, 1.36311340991091, 0.52005395609246, -1.99322761859398, 0.681872078965692, 2.67465341593641);
    addPath(P, 0.178054805603152, 1.22748137225951, 0.064635415579831, 0.843113409910906, 0.993640978710236, -1.79310541253395, 0.251641571454754, 2.19863258055732);
    addPath(P, 0.208202922038813, 1.11368441113305, -0.119901393232236, -0.152885332368672, 2.06432680267215, -0.774439324365845, 0.9890421038736, 1.1651435866182);
    addPath(P, 0.119794040645698, 1.33871095235714, 0.400098606767764, -0.373629239435118, 1.75305031308584, -0.506757666051433, 1.37533384982841, 0.90522106163889);
    addPath(P, 0.200744649781226, 1.24926751395852, 0.755091164112393, -0.00268870852003267, 1.74339999539584, -0.712255651205343, 0.942508484549711, 0.619291703500511);
    addPath(P, 0.357974887270042, 1.02953832290129, 1.1120673936905, -0.415498501755318, 1.79970946338367, -0.92405834419338, 1.4542944913284, -0.207940361854241);
    addPath(P, 0.106235115125885, 1.37606352480331, 1.39334696175522, -0.420497336506435, 1.59227563078897, -0.52, 1.47382850504935, 0.0509967387614102);
    addPath(P, 0.100000001490116, 1.57079637050629, 1.57079637050629, 0, 1.57079637050629, 0, 1.57079637050629, 0);
     return P;
}

int main(int argc, char *argv[]) {
    using S = double;
    using namespace mpl::demo;
    using Robot = FetchRobot<S>;
    using Config = typename Robot::Config;

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
    
    // Eigen::Matrix<S, 6, 1> L;
    // L << 1, 1, 1,   1, 1, 0.00001; // /1.570796326794897;

    // L << 0.01, 0.01, 0.01, 0.01, 0.01, 1.570796326794897;
    // S eps = L.minCoeff();
    // L = eps / L.array();

    // for (int iter = 0 ; iter<100 ; ++iter) {
    //     robot.setConfig(Robot::randomConfig(rng));
    //     if (robot.ik(ikTarget, L, eps)) {
    //         std::clog << "IK SOLVED after " << iter << std::endl;
    //         break;
    //     }
    // }
    
    // robot.updateArticulatedBlenderScript(bpy);

    std::vector<Config> path = makePath003<Config>();
    std::reverse(path.begin(), path.end());
        
    // robot.toCollisionGeometryBlenderScript(bpy);

    for (std::size_t i=0 ; i<path.size() ; ++i) {
        robot.setConfig(path[i]);
        robot.updateArticulatedBlenderScript(bpy);
        robot.keyframeInsert(bpy, i*20 + 1);
    }
    
    // for (int frame=0 ; frame<1 ; ++frame) {
    //     do {
    //         robot.setConfig(Robot::randomConfig(rng));
    //     } while (robot.selfCollision());

    //     robot.updateArticulatedBlenderScript(bpy);
    //     robot.keyframeInsert(bpy, frame*20 + 1);
    // }
    
    return EXIT_SUCCESS;
}
