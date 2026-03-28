// -*- coding: utf-8 -*-
/*
 * eskf_node.cpp  - Error-State Kalman Filter 15차원 GPS+IMU 융합
 */

#include <cmath>
#include <string>
#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace utm {
    static constexpr double kA=6378137.0, kF=1.0/298.257223563, kK0=0.9996;
    static constexpr double kE2=2*kF-kF*kF;
    inline std::pair<double,double> forward(double lat_deg,double lon_deg,int zone){
        double lat=lat_deg*M_PI/180,lon=lon_deg*M_PI/180;
        double lon0=((zone-1)*6-180+3)*M_PI/180;
        double e2=kE2,N=kA/sqrt(1-e2*sin(lat)*sin(lat));
        double T=tan(lat)*tan(lat),C=e2/(1-e2)*cos(lat)*cos(lat),A=cos(lat)*(lon-lon0);
        double M=kA*((1-e2/4-3*e2*e2/64-5*e2*e2*e2/256)*lat
                    -(3*e2/8+3*e2*e2/32+45*e2*e2*e2/1024)*sin(2*lat)
                    +(15*e2*e2/256+45*e2*e2*e2/1024)*sin(4*lat)
                    -(35*e2*e2*e2/3072)*sin(6*lat));
        double x=kK0*N*(A+(1-T+C)*A*A*A/6+(5-18*T+T*T+72*C-58*e2/(1-e2))*A*A*A*A*A/120)+500000;
        double y=kK0*(M+N*tan(lat)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                    +(61-58*T+T*T+600*C-330*e2/(1-e2))*A*A*A*A*A*A/720));
        if(lat_deg<0)y+=10000000.0;
        return{x,y};
    }
}

using Q4=Eigen::Vector4d;
static Q4 qnorm(const Q4&q){double n=q.norm();return n>1e-10?q/n:Q4(1,0,0,0);}
static Q4 qmul(const Q4&a,const Q4&b){
    return{a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
           a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
           a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
           a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};}
static Eigen::Matrix3d qrot(const Q4&q){
    Q4 qn=qnorm(q);double w=qn[0],x=qn[1],y=qn[2],z=qn[3];
    Eigen::Matrix3d R;
    R<<1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y),
       2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x),
       2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y);
    return R;}
static double qyaw(const Q4&q){
    Q4 qn=qnorm(q);double w=qn[0],x=qn[1],y=qn[2],z=qn[3];
    return atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
static Q4 rv2q(const Eigen::Vector3d&rv){
    double a=rv.norm();
    if(a<1e-10)return{1,rv[0]/2,rv[1]/2,rv[2]/2};
    Eigen::Vector3d ax=rv/a;double h=a/2;
    return{cos(h),sin(h)*ax[0],sin(h)*ax[1],sin(h)*ax[2]};}
static Eigen::Matrix3d skew(const Eigen::Vector3d&v){
    Eigen::Matrix3d S;S<<0,-v[2],v[1],v[2],0,-v[0],-v[1],v[0],0;return S;}
static double wrapA(double a){a=fmod(a+M_PI,2*M_PI);return a<0?a+M_PI:a-M_PI;}

class ESKF15{
public:
    static constexpr int NX=15;
    Eigen::Vector3d p{0,0,0},v{0,0,0},bg{0,0,0},ba{0,0,0};
    Q4 q{1,0,0,0};
    Eigen::Matrix<double,NX,NX> P=Eigen::Matrix<double,NX,NX>::Identity();
    double sa,sg,sab,sgb,sgp,sgv,sh;
    Eigen::Vector3d grav{0,0,-9.81};
    bool initialized=false;

    ESKF15(double sa_,double sg_,double sab_,double sgb_,
           double sgp_,double sgv_,double sh_,double g_)
        :sa(sa_),sg(sg_),sab(sab_),sgb(sgb_),sgp(sgp_),sgv(sgv_),sh(sh_){
        grav[2]=-g_;
        P.block<3,3>(0,0)*=100;P.block<3,3>(3,3)*=1;P.block<3,3>(6,6)*=0.1;
        P.block<3,3>(9,9)*=0.01;P.block<3,3>(12,12)*=0.01;}

    void init(const Eigen::Vector3d&p0,double yaw0){
        p=p0;v.setZero();q=Q4(cos(yaw0/2),0,0,sin(yaw0/2));
        bg.setZero();ba.setZero();initialized=true;}

    void predict(const Eigen::Vector3d&am,const Eigen::Vector3d&wm,double dt){
        if(!initialized||dt<=0||dt>1)return;
        Eigen::Vector3d a=am-ba,w=wm-bg;
        Eigen::Matrix3d R=qrot(q);
        Eigen::Vector3d pn=p+v*dt+0.5*(R*a+grav)*dt*dt;
        Eigen::Vector3d vn=v+(R*a+grav)*dt;
        Q4 qn=qnorm(qmul(q,rv2q(w*dt)));
        Eigen::Matrix<double,NX,NX> F=Eigen::Matrix<double,NX,NX>::Identity();
        F.block<3,3>(0,3)=Eigen::Matrix3d::Identity()*dt;
        F.block<3,3>(3,6)=-R*skew(a)*dt;
        F.block<3,3>(3,12)=-R*dt;
        F.block<3,3>(6,6)=Eigen::Matrix3d::Identity()-skew(w)*dt;
        F.block<3,3>(6,9)=-Eigen::Matrix3d::Identity()*dt;
        Eigen::Matrix<double,12,12> Qc=Eigen::Matrix<double,12,12>::Zero();
        Qc.block<3,3>(0,0)=Eigen::Matrix3d::Identity()*sa*sa;
        Qc.block<3,3>(3,3)=Eigen::Matrix3d::Identity()*sg*sg;
        Qc.block<3,3>(6,6)=Eigen::Matrix3d::Identity()*sgb*sgb;
        Qc.block<3,3>(9,9)=Eigen::Matrix3d::Identity()*sab*sab;
        Eigen::Matrix<double,NX,12> Fw=Eigen::Matrix<double,NX,12>::Zero();
        Fw.block<3,3>(3,0)=-R;Fw.block<3,3>(6,3)=-Eigen::Matrix3d::Identity();
        Fw.block<3,3>(9,6)=Fw.block<3,3>(12,9)=Eigen::Matrix3d::Identity();
        P=F*P*F.transpose()+Fw*(Qc*dt)*Fw.transpose();
        p=pn;v=vn;q=qn;}

    void correct(const Eigen::MatrixXd&H,const Eigen::MatrixXd&Rn,const Eigen::VectorXd&innov){
        Eigen::MatrixXd S=H*P*H.transpose()+Rn;
        Eigen::MatrixXd K=P*H.transpose()*S.inverse();
        Eigen::VectorXd dx=K*innov;
        p+=dx.segment<3>(0);v+=dx.segment<3>(3);
        q=qnorm(qmul(q,rv2q(dx.segment<3>(6))));
        bg+=dx.segment<3>(9);ba+=dx.segment<3>(12);
        auto IKH=Eigen::Matrix<double,NX,NX>::Identity()-K*H;
        P=IKH*P*IKH.transpose()+K*Rn*K.transpose();}

    void updatePos(const Eigen::Vector3d&pos,const Eigen::Matrix3d*cov=nullptr){
        Eigen::Matrix<double,3,NX> H=Eigen::Matrix<double,3,NX>::Zero();
        H.block<3,3>(0,0)=Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rn=cov?*cov:Eigen::Matrix3d::Identity()*sgp*sgp;
        correct(H,Rn,pos-p);}

    void updateVel(const Eigen::Vector3d&vel,const Eigen::Matrix3d*cov=nullptr){
        Eigen::Matrix<double,3,NX> H=Eigen::Matrix<double,3,NX>::Zero();
        H.block<3,3>(0,3)=Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Rn=cov?*cov:Eigen::Matrix3d::Identity()*sgv*sgv;
        correct(H,Rn,vel-v);}

    void updateHdg(double hdg_rad,double sig=-1){
        Q4 qn=qnorm(q);double w=qn[0],x=qn[1],y=qn[2],z=qn[3];
        double yp=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
        double d1=2*(w*z+x*y),d2=1-2*(y*y+z*z),den=d1*d1+d2*d2;
        if(den<1e-10)return;
        Eigen::RowVector4d dq;
        dq<<2*z*d2/den,2*y*d2/den,(2*x*d2+4*y*d1)/den,2*w*d2/den;
        Eigen::Matrix<double,4,3> QL;
        QL<<-x,-y,-z,w,-z,y,z,w,-x,-y,x,w;QL*=0.5;
        Eigen::Matrix<double,1,NX> H=Eigen::Matrix<double,1,NX>::Zero();
        H.segment<3>(6)=dq*QL;
        double s=sig>0?sig:sh;
        Eigen::Matrix<double,1,1> Rn,inn;
        Rn(0,0)=s*s;inn(0,0)=wrapA(hdg_rad-yp);
        correct(H,Rn,inn);}

    double yawDeg()const{return qyaw(q)*180/M_PI;}
    double covTrace()const{return P.trace();}
};

class ESKFNode : public rclcpp::Node{
public:
    ESKFNode():Node("eskf_node"){
        declare_parameter("utm_zone",52);
        declare_parameter("odom_frame","odom");
        declare_parameter("base_frame","base_footprint");
        declare_parameter("imu_topic","/handsfree/imu");
        declare_parameter("gps_fix_topic","/gnss_left/fix");
        declare_parameter("gps_vel_topic","/ublox_gps/fix_velocity");
        declare_parameter("dual_heading_topic","/dual_f9p/heading");
        declare_parameter("dual_heading_valid_topic","/dual_f9p/heading_valid");
        declare_parameter("dual_heading_accuracy_topic","/dual_f9p/heading_accuracy_deg");
        declare_parameter("sigma_accel",0.05);
        declare_parameter("sigma_gyro",0.005);
        declare_parameter("sigma_accel_bias",0.001);
        declare_parameter("sigma_gyro_bias",0.0001);
        declare_parameter("sigma_gps_pos",0.5);
        declare_parameter("sigma_gps_vel",0.1);
        declare_parameter("sigma_heading",0.02);
        declare_parameter("gravity",9.81);
        declare_parameter("init_heading_deg",0.0);
        declare_parameter("yaw_offset_deg",0.0);

        utm_zone_   =get_parameter("utm_zone").as_int();
        odom_frame_ =get_parameter("odom_frame").as_string();
        base_frame_ =get_parameter("base_frame").as_string();
        yaw_off_    =get_parameter("yaw_offset_deg").as_double()*M_PI/180;
        init_hdg_   =get_parameter("init_heading_deg").as_double()*M_PI/180;

        eskf_=std::make_unique<ESKF15>(
            get_parameter("sigma_accel").as_double(),
            get_parameter("sigma_gyro").as_double(),
            get_parameter("sigma_accel_bias").as_double(),
            get_parameter("sigma_gyro_bias").as_double(),
            get_parameter("sigma_gps_pos").as_double(),
            get_parameter("sigma_gps_vel").as_double(),
            get_parameter("sigma_heading").as_double(),
            get_parameter("gravity").as_double());

        auto sq=rclcpp::SensorDataQoS();
        auto tl=rclcpp::QoS(1).reliable().transient_local();

        imu_sub_=create_subscription<sensor_msgs::msg::Imu>(
            get_parameter("imu_topic").as_string(),sq,
            std::bind(&ESKFNode::imuCb,this,std::placeholders::_1));
        fix_sub_=create_subscription<sensor_msgs::msg::NavSatFix>(
            get_parameter("gps_fix_topic").as_string(),sq,
            std::bind(&ESKFNode::fixCb,this,std::placeholders::_1));
        vel_sub_=create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            get_parameter("gps_vel_topic").as_string(),sq,
            std::bind(&ESKFNode::velCb,this,std::placeholders::_1));
        const auto dh_topic = get_parameter("dual_heading_topic").as_string();
        const auto dv_topic = get_parameter("dual_heading_valid_topic").as_string();
        const auto da_topic = get_parameter("dual_heading_accuracy_topic").as_string();
        if (!dh_topic.empty())
            dh_sub_=create_subscription<std_msgs::msg::Float64>(
                dh_topic,10,std::bind(&ESKFNode::dhCb,this,std::placeholders::_1));
        if (!dv_topic.empty())
            dv_sub_=create_subscription<std_msgs::msg::Bool>(
                dv_topic,10,std::bind(&ESKFNode::dvCb,this,std::placeholders::_1));
        if (!da_topic.empty())
            da_sub_=create_subscription<std_msgs::msg::Float64>(
                da_topic,10,std::bind(&ESKFNode::daCb,this,std::placeholders::_1));

        odom_pub_  =create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered",10);
        hdg_pub_   =create_publisher<std_msgs::msg::Float64>("/eskf/heading_deg",10);
        cov_pub_   =create_publisher<std_msgs::msg::Float64>("/eskf/covariance_trace",10);
        orig_pub_  =create_publisher<geometry_msgs::msg::Point>("/utm_origin",tl);

        RCLCPP_INFO(get_logger(),"ESKF 노드 시작 (UTM zone=%d)",utm_zone_);}

private:
    void imuCb(const sensor_msgs::msg::Imu::SharedPtr m){
        double t=m->header.stamp.sec+m->header.stamp.nanosec*1e-9;
        Eigen::Vector3d a(m->linear_acceleration.x,m->linear_acceleration.y,m->linear_acceleration.z);
        Eigen::Vector3d w(m->angular_velocity.x,m->angular_velocity.y,m->angular_velocity.z);
        if(!eskf_->initialized){
            if(got_gps_){
                Q4 qi(m->orientation.w,m->orientation.x,m->orientation.y,m->orientation.z);
                double y0=qi.norm()>0.9?qyaw(qi)+yaw_off_:init_hdg_;
                eskf_->init(eskf_->p,y0);
                RCLCPP_INFO(get_logger(),"ESKF 초기화 yaw=%.1f deg",y0*180/M_PI);}
            last_t_=t;return;}
        double dt=last_t_>0?t-last_t_:0;last_t_=t;
        if(dt>0&&dt<0.5){
            eskf_->predict(a,w,dt);
            // IMU orientation으로 헤딩 보정 (자이로 바이어스 드리프트 방지)
            // sigma 10° - 정지 시 드리프트 억제, GPS 속도 헤딩(~5°)이 이동 시 우선
            Q4 qi(m->orientation.w,m->orientation.x,m->orientation.y,m->orientation.z);
            if(qi.norm()>0.9){
                eskf_->updateHdg(qyaw(qi)+yaw_off_,10.0*M_PI/180.0);
            }
            pub();}}

    void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr m){
        if(std::isnan(m->latitude)||std::isnan(m->longitude))return;
        auto[xg,yg]=utm::forward(m->latitude,m->longitude,utm_zone_);
        Eigen::Vector3d pos(xg-ox_,yg-oy_,0);
        if(!got_gps_){ox_=xg;oy_=yg;got_gps_=true;
            geometry_msgs::msg::Point pt;pt.x=xg;pt.y=yg;orig_pub_->publish(pt);
            RCLCPP_INFO(get_logger(),"UTM 원점 (%.3f,%.3f)",xg,yg);
            eskf_->p.setZero();return;}
        if(!eskf_->initialized)return;
        Eigen::Matrix3d cov=Eigen::Matrix3d::Identity()*eskf_->sgp*eskf_->sgp;
        if(m->position_covariance[0]>0){cov(0,0)=m->position_covariance[0];cov(1,1)=m->position_covariance[4];}
        eskf_->updatePos(pos,&cov);}

    void velCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr m){
        if(!eskf_->initialized)return;
        double vx=m->twist.twist.linear.x,vy=m->twist.twist.linear.y;
        Eigen::Vector3d vel(vx,vy,0);
        Eigen::Matrix3d cov=Eigen::Matrix3d::Identity()*eskf_->sgv*eskf_->sgv;
        if(m->twist.covariance[0]>0)cov(0,0)=cov(1,1)=m->twist.covariance[0];
        eskf_->updateVel(vel,&cov);
        double spd=hypot(vx,vy);
        if(spd>0.5&&!dual_valid_)
            // GPS VTG 속도는 ENU 절대 방향 - yaw_off_는 IMU 장착 오프셋이므로 적용 안 함
            eskf_->updateHdg(atan2(vy,vx),
                             std::max(5.0*M_PI/180/std::max(spd,0.5),0.01));}

    void dhCb(const std_msgs::msg::Float64::SharedPtr m){
        if(!eskf_->initialized)return;
        dual_hdg_=m->data*M_PI/180+yaw_off_;
        if(dual_valid_)eskf_->updateHdg(dual_hdg_,dual_sig_);}
    void dvCb(const std_msgs::msg::Bool::SharedPtr m){dual_valid_=m->data;}
    void daCb(const std_msgs::msg::Float64::SharedPtr m){
        dual_sig_=std::max(0.01,m->data)*M_PI/180;}

    void pub(){
        if(!eskf_->initialized)return;
        nav_msgs::msg::Odometry o;
        o.header.stamp=now();o.header.frame_id=odom_frame_;o.child_frame_id=base_frame_;
        o.pose.pose.position.x=eskf_->p[0];
        o.pose.pose.position.y=eskf_->p[1];
        o.pose.pose.position.z=eskf_->p[2];
        Q4 qq=qnorm(eskf_->q);
        o.pose.pose.orientation.w=qq[0];o.pose.pose.orientation.x=qq[1];
        o.pose.pose.orientation.y=qq[2];o.pose.pose.orientation.z=qq[3];
        o.twist.twist.linear.x=eskf_->v[0];o.twist.twist.linear.y=eskf_->v[1];
        for(int i=0;i<3;++i){
            o.pose.covariance[i*6+i]=eskf_->P(i,i);
            o.pose.covariance[(i+3)*6+i+3]=eskf_->P(i+6,i+6);}
        odom_pub_->publish(o);
        std_msgs::msg::Float64 hd,cv;
        hd.data=eskf_->yawDeg();cv.data=eskf_->covTrace();
        hdg_pub_->publish(hd);cov_pub_->publish(cv);}

    std::unique_ptr<ESKF15> eskf_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dh_sub_,da_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dv_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hdg_pub_,cov_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr orig_pub_;
    int utm_zone_;
    std::string odom_frame_,base_frame_;
    double yaw_off_,init_hdg_;
    double ox_=0,oy_=0;bool got_gps_=false;
    double last_t_=-1;
    bool dual_valid_=false;double dual_hdg_=0,dual_sig_=0.02;
};

int main(int argc,char**argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ESKFNode>());
    rclcpp::shutdown();return 0;}
