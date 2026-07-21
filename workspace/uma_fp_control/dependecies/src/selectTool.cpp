#include <dependecies/selectTool.hpp>

selectTool::selectTool()//Constructor
{
	ROS_INFO_STREAM("---selectTool---");
}

selectTool::~selectTool()
{
    ROS_INFO_STREAM("Leaving gently selectTool...");
}

void selectTool::computeTwrist(int type, double p_estimado, double tool_length, UMA_trans* tr){
    Eigen::MatrixXd desplazamiento(4,4);
    Eigen::MatrixXd desplazamientoFulcro(4,4);
    Eigen::MatrixXd rotacion(4,4);
    std::vector<double> DTTP;
    if (type == 1){
        //Herramienta en el Eje Z
        TCP = {0., 0, tool_length, 0., 0., 0.};
        DFP = {TCP[0],TCP[1], p_estimado};
    }
    else if(type == 2){
        //Herramienta en el Eje Y
        TCP = {0, tool_length, 0.034, -1.5708,0, 0.};
        DFP = {TCP[0], p_estimado, TCP[2]};
    }
    else if(type == 3){
        //Sin Herramienta
        TCP = {0, 0, 0, 0., 0., 0.};
        DFP = {0,0,0};
    }
    else if(type == 4){
        ROS_INFO_STREAM("-----azul----");
        TCP = {0.008, -0.05825, tool_length, 0.0, 0.0, 0.0};
        DFP = {TCP[0],TCP[1], p_estimado};
    }
    else if(type == 5){
        //Herramienta propia en metros
        ROS_INFO_STREAM("-----Vacum----");
        //TCP = {0, tool_length, 0.034, -1.5708,0, 0.};
        TCP = {0, 0.029, 0.463, 0,0, 0};
        DFP = {TCP[0],TCP[1], p_estimado,};
    }
    else if(type == 6){
        ROS_INFO_STREAM("-----camera----");
        //TCP = {0.008, -0.05825, tool_length, 0.0, 0.0, 0.0};
        TCP = {0.000211, 0.014648, 0.030114, 0,0, 0};
        DFP = {0,0,0};
    }
    else {
        //Defecto-Sin Herramienta
        TCP = {0, 0, 0, 0., 0., 0.};
        DFP = {0,0,0};
    }
    DTTP = {TCP[0],TCP[1], TCP[2]};
    desplazamiento = tr->desp(DTTP);
    desplazamientoFulcro = tr->desp(DFP);
    DFP = {DFP[0],DFP[1], DFP[2],0,0,0};
    rotacion = tr->rotZ(TCP[5]) * tr->rotY(TCP[4]) * tr->rotX(TCP[3]);
    E_T_TTP = desplazamiento * rotacion;
    E_T_Fp = desplazamientoFulcro * rotacion;
}
