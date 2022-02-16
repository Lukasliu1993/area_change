#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <string>
#include <iostream>
#include <fstream>
#include <cassert>
#include <vector>


using namespace std;
namespace dtail_simulation {
    class osmfile {
        public:
            int id;
            string s, fname; 
            ifstream infile;
            vector<string> split(const string& s, const string& c);
            int get_selfid();
            int get_refid();
            float get_x_y();
            int get_mode();
            float get_vel();
            void open_file();
            void get_node();
            void close_file();
            bool read_line(int n);
            bool contain_string(string t);
    };
}

