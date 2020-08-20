#include <iostream> 
#include <string>  
#include <vector> 
#include <ctime>
#include <chrono>
#include <Eigen/Geometry>

struct Measurement{
    Eigen::Vector3f position;
    float length;
    float width;
    float height;
    bool assoc;
};

struct Track{
    float x;
    float y;
    float l;
    float w;
    float h;
    int age;

    Track(){
        age = 0;
    }
};
class Tracker{
    std::vector<Measurement> measurements;

    std::vector<std::pair<Track,int>> Objects;
    public:
        //DA function
        Tracker(){

        }
        void setMeasurements(std::vector<Measurement> &m){
            measurements.clear();
            measurements = m;
        }

        void associateMeasurements(){
            float x_threshold = 1;
            float y_threshold = 1;
            float d_threshold = 1.2;
            int max_na = 2;
            std::vector<int> del_ids;

            for(int i=0; i< Objects.size();i++){
                float x_o = Objects[i].first.x;
                float y_o = Objects[i].first.y;
                bool f_assoc = false;

                for(int m = 0;m<measurements.size();m++){
                    if(measurements[m].assoc == false){
                        float x_m = measurements[m].position[0];
                        float y_m = measurements[m].position[1];
                        if(abs(x_o - x_m) < x_threshold && abs(y_o - y_m) < y_threshold){
                            float dist = (x_o - x_m)*(x_o - x_m) + (y_o - y_m)*(y_o - y_m);
                            if(dist < d_threshold){
                                measurements[m].assoc = true;
                                Objects[i].first.x = x_m;
                                Objects[i].first.y = y_m;
                                Objects[i].first.l = measurements[m].length;
                                Objects[i].first.w = measurements[m].width;
                                Objects[i].first.h = measurements[m].height;
                                Objects[i].first.age++;
                                f_assoc = true;
                                break;
                            }
                        }
                    }
                }
                if(f_assoc == false){

                    Objects[i].second = Objects[i].second + 1;
                    if(Objects[i].second > max_na){
                        del_ids.push_back(i);
                    }
                }
            }
            for(int i=del_ids.size()-1;i>=0;i--){
                Objects.erase(Objects.begin()+del_ids[i]);
            }

            for(int m=0;m<measurements.size();m++){
                if(measurements[m].assoc == false){
                    std::pair<Track,int> n_track;
                    n_track.first.x = measurements[m].position[0];
                    n_track.first.y = measurements[m].position[1];
                    n_track.first.l = measurements[m].length;
                    n_track.first.w = measurements[m].width;
                    n_track.first.h = measurements[m].height;
                    n_track.first.age = 1;
                    n_track.second = 0;
                    Objects.push_back(n_track);
                }
            }

        }

        void printTracks(){
            std::cout << "Number of tracks : " << Objects.size() <<std::endl;
            for(int i=0;i<Objects.size();i++){
                std::cout<<"x: "<< Objects[i].first.x << ' '<<"y: "<< Objects[i].first.y << ' ';
                std::cout<<"Age: "<< Objects[i].first.age << ' '<<"na: "<< Objects[i].second <<std::endl;
            }
        }

};