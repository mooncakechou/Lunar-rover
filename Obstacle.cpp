// 坎拍摄视频

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#ifdef CHRONO_COLLISION
    #include "chrono/collision/ChCollisionSystemChrono.h"
#endif

#ifdef CHRONO_IRRLICHT
    #include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

using namespace chrono;

ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

int main() {
    const std::string out_dir = GetChronoOutputPath() + "DEMO";

    const double wheel_r = 0.88, wheel_w = 0.5, wheel_density = 100, wheel_h = 1;

    const double car_x = 8, car_y = 3.6, car_z = 4.5, car_h = wheel_h + 2.5, car_density = 100;

    const double car_position_x = -5, car_position_y = car_h, car_position_z = 0;

    const double wheel_position_x1 = car_x / 2 + car_position_x, wheel_position_y = wheel_h,
                 wheel_position_z1 = car_z / 2 + 1, wheel_position_x2 = car_position_x,
                 wheel_position_x3 = -car_x / 2 + car_position_x,
                 wheel_position_z2 = car_z / 2 + 1.266;

    const double hinge1_x = car_position_x, hinge1_y = car_position_y - 1;
    const double hinge2_x = hinge1_x - 1.6, hinge2_y = car_position_y - 1.5;

    const double stick_r = 0.1, stick_density = 1000;

    const double stick_l = 1, l = 0.5;

    ChSystemNSC sys;

    auto sph_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    sph_mat->SetFriction(0.8);

    auto wheel_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    wheel_mat->SetFriction(0.8);

    auto ground_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    ground_mat_vis->SetKdTexture(GetChronoDataFile("textures/whitegreen.JPG"));

    auto ramp_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    ramp_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));

    // Create ground
    auto ground = std::make_shared<ChBodyEasyBox>(100, 0.1, 100, 1000, sph_mat, collision_type);
    ground->SetPos(ChVector<>(0, -0.05, 0));
    ground->SetBodyFixed(true);
    ground->GetVisualShape(0)->SetMaterial(0, ground_mat_vis);
    sys.Add(ground);

    // Create ramps
    auto ground_l1 = std::make_shared<ChBodyEasyBox>(1, 0.01, 2, 1000, sph_mat, collision_type);
    ground_l1->SetPos(ChVector<>(7, 1, -wheel_position_z2));
    ground_l1->SetBodyFixed(true);
    ground_l1->GetVisualShape(0)->SetMaterial(0, ramp_mat_vis);
    sys.Add(ground_l1);

    auto ground_l2 = std::make_shared<ChBodyEasyBox>(1.414, 0.01, 2, 1000, sph_mat, collision_type);
    ground_l2->SetPos(ChVector<>(6, 0.5, -wheel_position_z2));
    ground_l2->SetRot(Q_from_AngZ(CH_C_PI / 4));
    ground_l2->SetBodyFixed(true);
    ground_l2->GetVisualShape(0)->SetMaterial(0, ramp_mat_vis);
    sys.Add(ground_l2);

    auto ground_l3 = std::make_shared<ChBodyEasyBox>(1.414, 0.01, 2, 1000, sph_mat, collision_type);
    ground_l3->SetPos(ChVector<>(8, 0.5, -wheel_position_z2));
    ground_l3->SetRot(Q_from_AngZ(-CH_C_PI / 4));
    ground_l3->SetBodyFixed(true);
    ground_l3->GetVisualShape(0)->SetMaterial(0, ramp_mat_vis);
    sys.Add(ground_l3);

    auto ground_r1 = std::make_shared<ChBodyEasyBox>(1, 0.01, 2, 1000, sph_mat, collision_type);
    ground_r1->SetPos(ChVector<>(9.5, 1, wheel_position_z1));
    ground_r1->SetBodyFixed(true);
    ground_r1->GetVisualShape(0)->SetMaterial(0, ramp_mat_vis);
    sys.Add(ground_r1);

    auto ground_r2 = std::make_shared<ChBodyEasyBox>(1.414, 0.01, 2, 1000, sph_mat, collision_type);
    ground_r2->SetPos(ChVector<>(8.5, 0.5, wheel_position_z1));
    ground_r2->SetRot(Q_from_AngZ(CH_C_PI / 4));
    ground_r2->SetBodyFixed(true);
    ground_r2->GetVisualShape(0)->SetMaterial(0, ramp_mat_vis);
    sys.Add(ground_r2);

    auto ground_r3 = std::make_shared<ChBodyEasyBox>(1.414, 0.01, 2, 1000, sph_mat, collision_type);
    ground_r3->SetPos(ChVector<>(10.5, 0.5, wheel_position_z1));
    ground_r3->SetRot(Q_from_AngZ(-CH_C_PI / 4));
    ground_r3->SetBodyFixed(true);
    ground_r3->GetVisualShape(0)->SetMaterial(0, ramp_mat_vis);
    sys.Add(ground_r3);

    // Create car body
    auto car = std::make_shared<ChBodyEasyBox>(car_x, car_y, car_z, car_density, false, false);
    car->SetPos(ChVector<>(car_position_x, car_position_y, car_position_z));
    sys.Add(car);
    auto Mesh = chrono_types::make_shared<ChModelFileShape>();
    Mesh->SetFilename(GetChronoDataFile("models/yueqiuche/newbody.obj"));
    car->AddVisualShape(Mesh);

    // Cheate wheels and sticks
    ChVector<> wheel_positions[6] = {ChVector<>(wheel_position_x1, wheel_position_y, wheel_position_z1),
                                     ChVector<>(wheel_position_x2, wheel_position_y, wheel_position_z2),
                                     ChVector<>(wheel_position_x3, wheel_position_y, wheel_position_z1),
                                     ChVector<>(wheel_position_x1, wheel_position_y, -wheel_position_z1),
                                     ChVector<>(wheel_position_x2, wheel_position_y, -wheel_position_z2),
                                     ChVector<>(wheel_position_x3, wheel_position_y, -wheel_position_z1)};

    ChVector<> stick_positions[8] = {ChVector<>(hinge1_x, hinge1_y, car_z / 2),
                                     ChVector<>(hinge2_x, hinge2_y, car_z / 2 + 0.35),
                                     ChVector<>(hinge2_x, hinge2_y, car_z / 2 + 0.123),
                                     ChVector<>(hinge1_x, hinge1_y, -car_z / 2),
                                     ChVector<>(hinge2_x, hinge2_y, -car_z / 2 - 0.35),
                                     ChVector<>(hinge2_x, hinge2_y, -car_z / 2 - 0.123),
                                     ChVector<>((hinge1_x + hinge2_x) / 2, (hinge1_y + hinge2_y) / 2, car_z / 2),
                                     ChVector<>((hinge1_x + hinge2_x) / 2, (hinge1_y + hinge2_y) / 2, -car_z / 2)};

    std::vector<std::shared_ptr<ChBodyEasyCylinder>> stick_list;
    std::vector<std::shared_ptr<ChBody>> wheel_list;

    for (int i = 0; i < 6; i++) {
        auto wheel = std::make_shared<ChBody>();
        wheel->SetPos(wheel_positions[i]);
        wheel->SetMass(600);
        wheel->SetInertiaXX(ChVector<>(2, 2, 2));
        sys.Add(wheel);
        wheel_list.push_back(wheel);
        ChMatrix33<> Arot(chrono::Q_from_AngX(CH_C_PI / 2));
        wheel->GetCollisionModel()->ClearModel();
        wheel->GetCollisionModel()->AddCylinder(wheel_mat, wheel_r, wheel_r, wheel_w, ChVector<>(0, 0, 0), Arot);
        wheel->GetCollisionModel()->BuildModel();
        wheel->SetCollide(true);
        auto tireMesh = chrono_types::make_shared<ChModelFileShape>();
        if (i < 3) {
            tireMesh->SetFilename(GetChronoDataFile("models/yueqiuche/wr.obj"));
        } else {
            tireMesh->SetFilename(GetChronoDataFile("models/yueqiuche/wl.obj"));
        }
        wheel->AddVisualShape(tireMesh);
    }

    for (int i = 0; i < 8; i++) {
        auto stick = std::make_shared<ChBodyEasyCylinder>(stick_r, stick_l, stick_density, false, false);
        stick->SetPos(stick_positions[i]);
        sys.Add(stick);
        stick_list.push_back(stick);
        auto mesh = chrono_types::make_shared<ChModelFileShape>();
        if (i == 2)
            mesh->SetFilename(GetChronoDataFile("models/yueqiuche/gan3.obj"));
        if (i == 5)
            mesh->SetFilename(GetChronoDataFile("models/yueqiuche/gan6.obj"));
        if (i == 0)
            mesh->SetFilename(GetChronoDataFile("models/yueqiuche/gan1.obj"));
        if (i == 3)
            mesh->SetFilename(GetChronoDataFile("models/yueqiuche/gan4.obj"));
        if (i == 1)
            mesh->SetFilename(GetChronoDataFile("models/yueqiuche/gan2.obj"));
        if (i == 4)
            mesh->SetFilename(GetChronoDataFile("models/yueqiuche/gan5.obj"));
        stick->AddVisualShape(mesh);
    }

    /// Create constraints

    //
    for (int i = 0; i < 6; i++) {
        auto my_link = std::make_shared<ChLinkLockRevolute>();
        my_link->Initialize(stick_list[i], wheel_list[i], ChCoordsys<>(ChVector<>(wheel_positions[i])));
        sys.AddLink(my_link);
    }

    // Add motor
    auto speed_function = chrono_types::make_shared<ChFunction_Const>(1 * CH_C_PI);
    for (int i = 0; i < 2; i++) {
        auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        my_motor->Initialize(stick_list[2 + 3 * i], wheel_list[2 + 3 * i], ChFrame<>(wheel_positions[2 + 3 * i]));
        sys.AddLink(my_motor);
        my_motor->SetSpeedFunction(speed_function);
    }

    // Create connecting rods
    auto link_17 = std::make_shared<ChLinkLockLock>();
    link_17->Initialize(stick_list[0], stick_list[6], ChCoordsys<>(ChVector<>(hinge1_x, hinge1_y, car_z / 2)));
    sys.AddLink(link_17);

    auto link_23 = std::make_shared<ChLinkLockLock>();
    link_23->Initialize(stick_list[1], stick_list[2], ChCoordsys<>(ChVector<>(hinge2_x, hinge2_y, car_z / 2)));
    sys.AddLink(link_23);

    auto link_48 = std::make_shared<ChLinkLockLock>();
    link_48->Initialize(stick_list[3], stick_list[7], ChCoordsys<>(ChVector<>(hinge1_x, hinge1_y, -car_z / 2)));
    sys.AddLink(link_48);

    auto link_56 = std::make_shared<ChLinkLockLock>();
    link_56->Initialize(stick_list[4], stick_list[5], ChCoordsys<>(ChVector<>(hinge2_x, hinge2_y, -car_z / 2)));
    sys.AddLink(link_56);

    auto link_27 = std::make_shared<ChLinkLockRevolute>();
    link_27->Initialize(stick_list[1], stick_list[6], ChCoordsys<>(ChVector<>(hinge2_x, hinge2_y, car_z / 2)));
    sys.AddLink(link_27);

    auto link_58 = std::make_shared<ChLinkLockRevolute>();
    link_58->Initialize(stick_list[4], stick_list[7], ChCoordsys<>(ChVector<>(hinge2_x, hinge2_y, -car_z / 2)));
    sys.AddLink(link_58);

    // Craete constraints for rod with car body
    auto link_1 = std::make_shared<ChLinkLockRevolute>();
    link_1->Initialize(car, stick_list[0], ChCoordsys<>(ChVector<>(hinge1_x, hinge1_y, car_z / 2)));
    sys.AddLink(link_1);

    auto link_4 = std::make_shared<ChLinkLockRevolute>();
    link_4->Initialize(car, stick_list[3], ChCoordsys<>(ChVector<>(hinge1_x, hinge1_y, -car_z / 2)));
    sys.AddLink(link_4);

    /// Create joint constraints for top rod with rover body
    auto tr = std::make_shared<ChBodyEasyCylinder>(stick_r, car_z, 100, false, false);
    tr->SetPos(ChVector<>(car_position_x - l, car_position_y + car_y / 2, car_position_z));
    tr->SetRot(Q_from_AngX(CH_C_PI / 2));
    sys.Add(tr);

    auto revo_link = std::make_shared<ChLinkLockRevolute>();
    revo_link->Initialize(car, tr,
                          ChCoordsys<>(ChVector<>(car_position_x - l, car_position_y + car_y / 2, car_position_z)));
    sys.AddLink(revo_link);

    /// Create joint constraints for top left/right rods with top rod
    auto lr = std::make_shared<ChBodyEasyCylinder>(stick_r, stick_l, 100, false, false);
    lr->SetPos(ChVector<>(car_position_x, car_position_y, -car_z / 2));
    sys.Add(lr);

    auto lrf = std::make_shared<ChBodyEasyCylinder>(stick_r, l, 100, false, false);
    lrf->SetPos(ChVector<>(car_position_x - l / 2, car_position_y + car_y / 2, -car_z / 2));
    lrf->SetRot(Q_from_AngZ(CH_C_PI / 2));
    sys.Add(lrf);

    auto rr = std::make_shared<ChBodyEasyCylinder>(stick_r, stick_l, 100, false, false);
    rr->SetPos(ChVector<>(car_position_x, car_position_y, car_z / 2));
    sys.Add(rr);

    auto rrf = std::make_shared<ChBodyEasyCylinder>(stick_r, l, 100, false, false);
    rrf->SetPos(ChVector<>(car_position_x - l / 2, car_position_y + car_y / 2, car_z / 2));
    rrf->SetRot(Q_from_AngZ(CH_C_PI / 2));
    sys.Add(rrf);

    auto lt_link = std::make_shared<ChLinkLockSpherical>();
    lt_link->Initialize(lrf, tr, ChCoordsys<>(ChVector<>(car_position_x - l, car_position_y + car_y / 2, -car_z / 2)));
    sys.AddLink(lt_link);

    auto rt_link = std::make_shared<ChLinkLockSpherical>();
    rt_link->Initialize(rrf, tr, ChCoordsys<>(ChVector<>(car_position_x - l, car_position_y + car_y / 2, car_z / 2)));
    sys.AddLink(rt_link);

    auto lf_link = std::make_shared<ChLinkLockSpherical>();
    lf_link->Initialize(lrf, lr, ChCoordsys<>(ChVector<>(car_position_x, car_position_y + car_y / 2, -car_z / 2)));
    sys.AddLink(lf_link);

    auto rf_link = std::make_shared<ChLinkLockSpherical>();
    rf_link->Initialize(rrf, rr, ChCoordsys<>(ChVector<>(car_position_x, car_position_y + car_y / 2, car_z / 2)));
    sys.AddLink(rf_link);

    auto lc_link = std::make_shared<ChLinkLockLock>();
    lc_link->Initialize(lr, stick_list[3], ChCoordsys<>(ChVector<>(hinge1_x, hinge1_y, -car_z / 2)));
    sys.AddLink(lc_link);

    auto rc_link = std::make_shared<ChLinkLockLock>();
    rc_link->Initialize(rr, stick_list[0], ChCoordsys<>(ChVector<>(hinge1_x, hinge1_y, car_z / 2)));
    sys.AddLink(rc_link);

    // visualization
    ChVisualSystemIrrlicht vis;

    vis.SetWindowSize(1920, 1080);
    vis.SetWindowTitle("A simple project template");
    vis.Initialize();
    //vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    int cameraid = vis.AddCamera(ChVector<>(20, 10, -10), ChVector<>(car_position_x, car_position_y, car_position_z));

    vis.AttachSystem(&sys);

    sys.SetSolverType(ChSolver::Type::PSOR);
    sys.SetSolverMaxIterations(50);

    // Simulation loop
    ChRealtimeStepTimer rt;
    double step_size = 1e-4;
    long long int step = 0;

    while (vis.Run()) {
        vis.BeginScene();
        vis.Render();
        vis.EndScene();

        ChVector<> targetPos = car->GetPos();
        ChVector<> cameraPos = targetPos + ChVector<>(15, 5, 12);
        vis.SetCameraPosition(cameraid, cameraPos);
        vis.SetCameraTarget(cameraid, targetPos);

        std::string rover_dir = out_dir;
        std::string filename;

        if (step % 1000 == 0) {
            /*for (int i = 1; i < 7; i++) {
                auto body = wheel_list[i - 1];
                ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
                ChVector<> pos = ref_frame.GetPos();
                ChQuaternion<> rot = ref_frame.GetRot();
                ChVector<> vel = body->GetPos_dt();

                std::string delim = ",";
                filename = rover_dir + "/wheel_pos_rot" + std::to_string(i) + ".csv";
                std::ofstream file;
                if (sys.GetChTime() > 0)
                    file.open(filename, std::fstream::app);
                else {
                    file.open(filename);
                    file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1"
                         << delim << "q2" << delim << "q3" << std::endl;
                }

                file << sys.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
                     << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << std::endl;

                file.close();
            }
            for (int i = 1; i < 7; i++) {
                auto body = stick_list[i - 1];
                ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
                ChVector<> pos = ref_frame.GetPos();
                ChQuaternion<> rot = ref_frame.GetRot();
                ChVector<> vel = body->GetPos_dt();

                std::string delim = ",";
                filename = rover_dir + "/stick_pos_rot" + std::to_string(i) + ".csv";
                std::ofstream file;
                if (sys.GetChTime() > 0)
                    file.open(filename, std::fstream::app);
                else {
                    file.open(filename);
                    file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1"
                         << delim << "q2" << delim << "q3" << std::endl;
                }

                file << sys.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
                     << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << std::endl;

                file.close();
            }*/

            auto body = car;
            ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> pos = ref_frame.GetPos();
            ChQuaternion<> rot = ref_frame.GetRot();
            ChVector<> vel = body->GetPos_dt();

            std::string delim = ",";
            filename = rover_dir + "/body_pos_rot" + ".csv";
            std::ofstream file;
            if (sys.GetChTime() > 0)
                file.open(filename, std::fstream::app);
            else {
                file.open(filename);
                file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1"
                     << delim << "q2" << delim << "q3" << std::endl;
            }

            file << sys.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
                 << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << std::endl;

            file.close();
        }
        step++;
        sys.DoStepDynamics(step_size);

        ////std::cout << std::endl;
        ////auto frc = mixer->GetAppliedForce();
        ////auto trq = mixer->GetAppliedTorque();
        ////std::cout << sys.GetChTime() << "  force: " << frc << "  torque: " << trq << std::endl;
        ////auto c_frc = mixer->GetContactForce();
        ////auto c_trq = mixer->GetContactTorque();
        ////std::cout << sys.GetChTime() << "  ct force: " << c_frc << "  ct torque: " << c_trq << std::endl;

        rt.Spin(step_size);
    }

    return 0;
}
