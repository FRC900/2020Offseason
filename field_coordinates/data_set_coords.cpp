#include <iostream>
#include <cctype>
#include <string>
#include <vector>

struct point {
  double metric_x = -1;
  double metric_y = -1;
} point_1;
char min_y = 'A';

bool user_friendly (int x_, char y_){
  bool answer = true;
  char max_y = 'F';
  int min_x = 0;
  int max_x = 12;
  std::vector<std::string> bad_coords = {"A2","A5","A11","E2","E5","E11","C1","C2","C4","C5","C6","C7","C8","C9","C10","C11"};
  std::string whole_point = y_ + std::to_string(x_);
  if (x_ < min_x || x_ > max_x){
    answer = false;
  }
  else if (y_ < min_y || y_ > max_y){
    answer = false;
  }
  else {
    for (int i = 0; i < bad_coords.size(); i++) {
      if (whole_point == bad_coords[i]) {
        answer = false;
      }
    }
  }
  return answer;
}

bool convert_to_metric(int x_coord, char y_coord, point& input){
  int max_y_dist = 381;
  double grid_dist = 76.2;
  if (user_friendly(x_coord, y_coord)){
    input.metric_x = (x_coord*grid_dist);
    input.metric_y = (max_y_dist-(y_coord-min_y)*grid_dist);
    std::cout<<point_1.metric_x<<" cm, "<<point_1.metric_y<<" cm"<<std::endl;
    return user_friendly(x_coord, y_coord);
  }
  else {
    std::cout<<"Invalid input. Please retry with the format: <letter a-f> <number 0-12>"<<std::endl<<"Points A2, A5, A11, C1, C2, C4, C5, C6, C7, C8, C10, C11, E2, E5, and E11 do not exist."<<std::endl;
    return user_friendly(x_coord, y_coord);
  }
}

int main(int argc, char const *argv[]){
  char y;
  int x;
  std::cout<<"Input field coordinates"<<std::endl;
  std::cin>>y;
  std::cin>>x;
  y = toupper(y);
  convert_to_metric(x, y, point_1);
}
