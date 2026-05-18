#include "gcode_parser.h"
#include "global_def.h"
#include "axis_ctrl.h"
#include <math.h>
#define PI 3.14159265358979323846
#define ARC_SEGMENT_LENGTH_MM 0.5 // 圆弧插补时的分段长度，单位mm

GCodeState_t g_state = {0.0, 0.0, 0.0, 1000.0, 1}; // 全局G-code状态变量，初始值为0
ParserControl_t g_parser_ctrl = {"", 0, 0, 0}; // 全局G-code解析控制变量，初始值为未运行、未暂停、未请求中止
extern void api_push_trajectory(double target_pos[AXIS_NUM],double speed,double acc,double dec);
extern void api_push_continuous_segment(double val_x,double val_y,double val_z,double speed_sec);

const char* skip_spaces(const char* str)
{
    while(*str==' '||*str=='\t') str++;
    return str;
}

int parse_gcode_line(const char *gcode_line)
{
    char buffer[128];
    strncpy(buffer, gcode_line, sizeof(buffer));
    
    int is_G00=0,is_G01=0,is_G02=0,is_G03=0;
    int has_move=0;
    //int has_x=0,has_y=0,has_z=0;
    int has_axis[AXIS_NUM]={0};
  
    double val_axis[AXIS_NUM]={0};
    //double val_x=0,val_y=0,val_z=0;
    double offset_i=0.0,offset_j=0.0;// 圆弧中心相对起点的偏移量，G02/G03专用


    char *p=buffer;
    while(*p!='\0'){
        p=(char*)skip_spaces(p);
        if(*p=='\0') break;

        char letter=toupper(*p);
        p++;

        double value=strtod(p, &p); // 解析数字，p会更新到数字后面的位置

        switch(letter){
            case 'G':
                if(value==0.0) is_G00=1;
                else if(value==1.0) is_G01=1;
                else if(value==90.0) g_state.is_absolute=1;
                else if(value==91.0) g_state.is_absolute=0;
                else if(value==2.0) is_G02=1;
                else if(value==3.0) is_G03=1;
                  
                break;
            case 'X':val_axis[0]=value;has_move=1;has_axis[0]=1; break;
            case 'Y':val_axis[1]=value;has_move=1;has_axis[1]=1; break;
            case 'Z':val_axis[2]=value;has_move=1;has_axis[2]=1; break;
            case 'A':val_axis[3]=value;has_move=1;has_axis[3]=1; break;
            case 'B':val_axis[4]=value;has_move=1;has_axis[4]=1; break;
            case 'F':g_state.feedrate_mm_min=value;break;
            case 'I':offset_i=value;break;
            case 'J':offset_j=value;break;
            default:
                // 其他命令暂不处理
                break;
        }

    }

    if(is_G00||is_G01||is_G02||is_G03||has_move){

        double target_pos[AXIS_NUM];
        double start_pos[AXIS_NUM];

        for(int i=0;i<AXIS_NUM;i++){
            start_pos[i]=g_state.current_pos[i];
            if(g_state.is_absolute){
                target_pos[i]=has_axis[i]?val_axis[i]:g_state.current_pos[i];
            }else{
                target_pos[i]=g_state.current_pos[i]+(has_axis[i]?val_axis[i]:0);
            }

        }

        double run_speed_mm=is_G00?RAPID_SPEED_MM_MIN:g_state.feedrate_mm_min;



        if(is_G02||is_G03){
            generate_arc_trajectory(start_pos,
                                    target_pos,
                                    offset_i, offset_j,
                                    is_G02, run_speed_mm);
            // 圆弧命令处理后直接返回，当前状态更新在插补完成后由规划器同步光标时完成
           
        } else{
           
            double speed_mm_sec=run_speed_mm/60.0;

            
            api_push_trajectory(target_pos,speed_mm_sec,DEFAULT_ACC,DEFAULT_DEC);

        }

        for(int i=0;i<AXIS_NUM;i++){
            g_state.current_pos[i]=target_pos[i];
        }

        printf("[Parser] 解析命令: %s -> 目标 (%.3f, %.3f, %.3f) mm, 速度 %.1f mm/min\n", 
                buffer, target_pos[0], target_pos[1], target_pos[2], run_speed_mm);


        
    }
}

OSAL_THREAD_FUNC parser_thread_func(void *arg){
    char line_buffer[256];
    
    while(1){

        //1.
        if(g_parser_ctrl.is_running==1){


            while(!g_all_axis_op_ready){
                osal_usleep(100000); // 等待所有轴准备就   
            }
            
            printf("[Parser] Processing file: %s\n", g_parser_ctrl.filepath);
            FILE *fp=fopen(g_parser_ctrl.filepath,"r");
            if(fp==NULL){
                printf("[Parser错误] 无法打开文件: %s\n", g_parser_ctrl.filepath);
                g_parser_ctrl.is_running=0;
                continue;
            }

            while(!is_trajectory_finished()){
                osal_usleep(100000); // 等待当前轨迹执行完成，检查频率为100ms
            }

            api_sync_planner_cursor(); // 同步规划器光标，确保新轨迹从当前状态开始

            for(int i=0;i<AXIS_NUM;i++){
                g_state.current_pos[i]=api_get_cursor(i);
            }
            //g_state.current_x_mm=api_get_cursor_x();
            //g_state.current_y_mm=api_get_cursor_y();
            //g_state.current_z_mm=api_get_cursor_z();

            while(fgets(line_buffer,sizeof(line_buffer),fp)!=NULL){
                if(g_parser_ctrl.abort_request){
                    printf("[Parser] 中止请求已收到，停止解析文件: %s\n", g_parser_ctrl.filepath);
                    break;
                }
                // 暂停检查
                while(g_parser_ctrl.is_paused){
                    osal_usleep(100000); // 暂停时每100ms检查一次状态
                }
                // 解析当前行G-code命令
                parse_gcode_line(line_buffer);
            }
            fclose(fp);
            api_flush_planner();
            printf("[Parser] 文件处理完成: %s\n", g_parser_ctrl.filepath);
            g_parser_ctrl.is_running=0; // 处理完成后重置状态
            g_parser_ctrl.abort_request=0; // 重置中止请求
        }

        osal_usleep(50000); // 主循环每50ms检查一次状态
    }
}

void generate_arc_trajectory(double start_pos[AXIS_NUM],double end_pos[AXIS_NUM], 
                             double i_offset, double j_offset,
                             int is_CW,double feedrate_mm_min)
{
    // 1.计算圆心坐标
    double cx=start_pos[0]+i_offset;
    double cy=start_pos[1]+j_offset;

    //2.计算半径
    double radius=hypot(start_pos[0]-cx,start_pos[1]-cy);
    if(radius<0.001)return;// 半径过小，直接当直线处理

    //3.计算起始和结束点的角度
    double theta_start=atan2(start_pos[1]-cy,start_pos[0]-cx);
    double theta_end=atan2(end_pos[1]-cy,end_pos[0]-cx);

    //4.根据顺逆时针调整结束角度，使其相对于起始角度在正确的范围内
    if(is_CW){ //G02 顺时针
        if(theta_end>=theta_start) theta_end-=2*PI;
    }else{  //G03 逆时针
        if(theta_end<=theta_start) theta_end+=2*PI;
    }

    double total_angle=theta_end-theta_start;

    //5.根据总角度和半径计算圆弧长度，并确定分段数
    double arc_length=fabs(total_angle)*radius;
    int num_segments=(int)ceil(arc_length/ARC_SEGMENT_LENGTH_MM);
    if(num_segments<1) num_segments=1;

    double angle_step=total_angle/num_segments;
    //double z_step=(end_z-start_z)/num_segments; // Z轴线性插补
    double speed_mm_sec=(feedrate_mm_min/60.0);

    double next_pos[AXIS_NUM];
    //6.生成每个插补点的坐标并推送到轨迹规划器
    for(int i=1;i<=num_segments;i++){
        double theta=theta_start+i*angle_step;

        next_pos[0]=cx+radius*cos(theta);
        next_pos[1]=cy+radius*sin(theta);
       
        double progress_ratio=(double)i/num_segments;
        for(int j=2;j<AXIS_NUM;j++){
            next_pos[j]=start_pos[j]+(end_pos[j]-start_pos[j])*progress_ratio;
        }

        if(i==num_segments){
            for(int j=0;j<AXIS_NUM;j++){
                next_pos[j]=end_pos[j];
            }
        }
        api_push_trajectory(next_pos,speed_mm_sec,DEFAULT_ACC,DEFAULT_DEC);
    }
    printf("[Parser] 生成了 %d 个圆弧插补点\n", num_segments);
}