#include <speed_area/osmfile.h>
#include <speed_area/speed_area.h>
using namespace std;
namespace dtail_simulation  
{
    //osmfile类中定义了对OSM文件的一些处理方法

    //分割字符串方法
    vector<string> osmfile::split(const string& s, const string& c)
    {
        vector<string> v;
        string::size_type pos1, pos2;
        pos2 = s.find(c);
        pos1 = 0;
        while(string::npos != pos2)
        {
            v.push_back(s.substr(pos1, pos2-pos1));
            pos1 = pos2 + c.size();
            pos2 = s.find(c, pos1);
        }
        if(pos1 != s.length())
            v.push_back(s.substr(pos1));
        return v;
    }
    //打开文件
    void osmfile::open_file()
    {
        infile.open(fname.data());  
    }
    //关闭文件
    void osmfile::close_file()
    {
        infile.close();
    }
    //读取后续第n行的内容，第n行的内容会更新进类的成员s内
    bool osmfile::read_line(int n)
    {
        int i = 0;;
        while(i < n)
        {
            if(getline(infile, s))
            {
                i++;
            }
            else
            {
                return false;
            }
        }
        return true;
    }
    //在OSM中定义元素的行中，读取ID号
    int osmfile::get_selfid()
    {
        vector<string> ele = split(s, "\"");
        return atoi(ele[1].c_str());
    }
    //在OSM中定义元素子结点的行中，读取子节点的ID号
    int osmfile::get_refid()
    {
        vector<string> ele = split(s, "ref=");
        ele = split(ele[1], "\"");
        return atoi(ele[1].c_str());
    }
    //在OSM中定义位置坐标的行中，读取子节点的x或y坐标
    float osmfile::get_x_y()
    {
        vector<string> ele = split(s, "v=");
        ele = split(ele[1], "\"");
        return atof(ele[1].c_str());
    }
    //在OSM中定义清扫模式属性的行中，读取清扫模式
    int osmfile::get_mode()
    {
        vector<string> ele = split(s, "v=");
        ele = split(ele[1], "\"");
        return atoi(ele[1].c_str());
    }
    //在OSM中定义速度上限的行中，读取速度上限
    float osmfile::get_vel()
    {
        vector<string> ele = split(s, "v=");
        ele = split(ele[1], "\"");
        ele = split(ele[1], "km");
        return atof(ele[0].c_str());
    }
    //判断成员s的字符串中是否含有t字符
    bool osmfile::contain_string(string t)
    {

        return s.find(t) != string::npos;
    }


    speed_area::speed_area() {};
    template <class T>
    //寻找id == name的元素，在其集合中的索引，元素可以为节点、线段或区域
    int speed_area::findidx(vector<T> &box, int name)
    {
        for (int i = 0; i < box.size(); i++)
        {
            if(box[i].id == name)
            return i;
        }
        return -1;
    }
    //定义了readarea方法，用来从path.osm中读取每个区域的信息放到arealist中
    vector<node_area> speed_area::readarea(string file)
    {
        vector<corner> corner_box;
        vector<way> way_box;
        vector<node_area> area_box;

        osmfile osmfile_;
        osmfile_.fname = file;
        osmfile_.open_file();
        //读取角点集合
        while(osmfile_.read_line(1))
        {
            if(osmfile_.contain_string("node id="))
            {
                corner t;
                t.id = osmfile_.get_selfid();
                ROS_INFO("cid: %d", t.id );
                osmfile_.read_line(2);
                t.x = osmfile_.get_x_y();
                ROS_INFO("x: %f", t.x );
                osmfile_.read_line(1);
                t.y = osmfile_.get_x_y();
                ROS_INFO("y: %f", t.y );
                corner_box.push_back(t);
            }
        }
        osmfile_.close_file();
        osmfile_.open_file();
        //读取线段集合
        while(osmfile_.read_line(1))
        {
            if(osmfile_.contain_string("way id="))
            {
                way t;
                t.id = osmfile_.get_selfid();
                ROS_INFO("wid: %d", t.id );
                osmfile_.read_line(1);
                while(!osmfile_.contain_string("tag k"))
                {
                    int idx = findidx(corner_box, osmfile_.get_refid());
                    if(idx >= 0)
                    {
                        t.space.push_back(corner_box[idx]);
                    }
                    osmfile_.read_line(1);
                }
                way_box.push_back(t); 
            }  
        }
        osmfile_.close_file();
        osmfile_.open_file();
        //读取区域集合
        while(osmfile_.read_line(1))
        {
            if(osmfile_.contain_string("relation id="))
            {
                node_area t;
                t.id = osmfile_.get_selfid();
                osmfile_.read_line(3);
                t.speed = osmfile_.get_vel();
                osmfile_.read_line(1);
                t.mode = osmfile_.get_mode();
                osmfile_.read_line(1);
                int idx = findidx(way_box, osmfile_.get_refid());
                for(int i = 0; i < way_box[idx].space.size(); i++)
                {
                    t.space.push_back(way_box[idx].space[i]);
                }
                osmfile_.read_line(1);     
                idx = findidx(way_box, osmfile_.get_refid());
                for(int i = way_box[idx].space.size() - 1; i >= 0; i--)
                {
                    t.space.push_back(way_box[idx].space[i]);
                }
                t.space.push_back(t.space[0]);//isinarea方法要求区域角点首尾闭合
                area_box.push_back(t); 
            }  
        }
        return area_box;
    }
    //定义了isinarea方法，用来判断点是否在多边形内部
    bool speed_area::isinarea(node_area area, float x, float y)
    {
        vector<float> lnglist,latlist;
        float maxlng, minlng, maxlat, minlat;
        for (int i = 0; i < area.space.size()-1; i++)
        {
            lnglist.push_back(area.space[i].x);
            latlist.push_back(area.space[i].y);
            maxlng = *max_element(lnglist.begin(), lnglist.end());
            maxlat = *max_element(latlist.begin(), latlist.end());
            minlng = *min_element(lnglist.begin(), lnglist.end());
            minlat = *min_element(latlist.begin(), latlist.end());
        }
        if (x > maxlng || x < minlng || y > maxlat || y < minlat)
        {
            return false;
        }
        int count = 0;
        corner point1 = area.space[0];
        for (int i = 1; i < area.space.size(); i++)
        {
            corner point2 = area.space[i];
            if ((x == point1.x && y == point1.y) || (x == point2.x && y == point2.y))
            {
                return true;
            }
            if ((point1.y < y && point2.y >= y) || (point1.y >= y && point2.y < y))
            {
                float point12lng = point2.x - (point2.y - y) * (point2.x - point1.x)/(point2.y - point1.y);
                if (point12lng == x)
                {
                    return true;
                }
                if (point12lng < x)
                {
                    count++;
                }
            }
            point1 = point2;
        }
        if (count % 2 == 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    //根据该区域的属性进行决策
    void speed_area::set_areaparam(node_area area)
    {
        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::Config conf; 
        dynamic_reconfigure::DoubleParameter vparam;
        vparam.name = "max_vel_x";
        vparam.value = (double) area.speed;
        conf.doubles.push_back(vparam);
        srv_req.config = conf;
        ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp);
        if(clean_mode_change_){
            switch (area.mode)
            {
            case 0:
                ros::param::set("wash_start_",0);
                ros::param::set("brush_start_",0);
                break;
            case 1:
                ros::param::set("wash_start_",0);
                ros::param::set("brush_start_",1);
                break;
            case 2:
                ros::param::set("brush_start_",0);
                ros::param::set("wash_start_",1);
                break;
            default:
                break;
            }
        }
    }
    void speed_area::parse_msg(const ros::TimerEvent &)
    {
        tf::StampedTransform local_transform;
        if(tf_.canTransform(map_frame_, base_frame_, ros::Time()))
        {
            //根据TF读取车辆当前位置
            tf_.lookupTransform(map_frame_, base_frame_, ros::Time(), local_transform);
            double pos_x = local_transform.getOrigin().x();
            double pos_y = local_transform.getOrigin().y();
            //当车辆驶出当前区域时，搜索车辆目前的所处区域
            if(!isinarea(arealist[cur_areaid], pos_x, pos_y) || !initialed)
            {   
                for (int i = 0; i < arealist.size(); i++)
                {
                    if(isinarea(arealist[i], pos_x, pos_y))
                    {
                        //根据该区域的属性进行决策
                        set_areaparam(arealist[i]);
                        cur_areaid = i;
                        initialed = true;
                        break;
                    }
                }
            }
        }
    }
    void speed_area::run() 
    {
        ros::NodeHandle node;
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("map_frame", map_frame_, "/map");
        private_nh.param<std::string>("car_frame", base_frame_, "/base_link");
        private_nh.param<std::string>("filename", filename, "");
        private_nh.param<bool>("clean_mode_change", clean_mode_change_, false);
        private_nh.param<float>("rate", rate, 1.0);
        std::string path =ros::package::getPath("speed_area") + "/data/" + filename + ".osm";
        ROS_INFO("path: %s", path.c_str());
        arealist = readarea(path);
        ros::Timer area_timer = private_nh.createTimer(ros::Duration(rate), &speed_area::parse_msg, this);
        ros::spin(); 
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_area");
    dtail_simulation::speed_area speed_area_;
    speed_area_.run();
    return 0;
}