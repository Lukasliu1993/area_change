#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>

using namespace std;
namespace dtail_simulation {
    //定义了角点类型
    class corner {
        public:
            int id;
            float x,y; 
    };
    //定义了线段类型，线段由角点组成
    class way {
        public:
            int id;
            vector<corner> space; 
    };
    //定义了区域类型，区域由角点组成
    class node_area {
        public:
            int id;
            float speed;
            int mode;
            vector<corner> space; 
    };
    class speed_area {

        public:
            speed_area();
            ~speed_area() = default;
            template <class T>
            int findidx(vector<T> &box, int name);
            vector<node_area> readarea(string file);
            bool isinarea(node_area area, float x, float y);
            void set_areaparam(node_area area);
            void parse_msg(const ros::TimerEvent &);
            void run();


        private:
            std::string pub_topic, filename, base_frame_, map_frame_;
            float rate;
            vector<node_area> arealist;
            ros::NodeHandle nh_;
            tf::TransformListener tf_;
            int cur_areaid = 0;
            bool initialed = false;
            bool clean_mode_change_;
    };
}


