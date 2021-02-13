#include <iostream>
#include <cctype>
#include <string>

struct point {
  int x_coord;
  char y_coord;
  double metric_x = -1;
  double metric_y = -1;
  std::string whole_point = y_coord + std::to_string(x_coord);
  std::string bad_coords[16] = {"A2","A5","A11","E2","E5","E11","C1","C2","C4","C5","C6","C7","C8","C9","C10","C11"};
  bool is_valid = true;
} point_1;
char min_y = 'A';
char max_y = 'F';
int min_x = 0;
int max_x = 12;

bool user_friendly (point& input){
  bool answer = true;
  if (input.x_coord < min_x || input.x_coord > max_x){
    answer = false;
  }
  else if (input.y_coord < min_y || input.y_coord > max_y){
    answer = false;
  }
  else {
    for (int i = 0; i <= sizeof(input.bad_coords); i++) {
      if (input.whole_point == input.bad_coords[i]) {
        answer = false;
      }
    }
  }
  return answer;
}

bool convert_to_metric(point& input){
  int x_ = input.x_coord;
  char y_ = input.y_coord;
  int max_y_dist = 381;
  double grid_dist = 76.2;
  user_friendly(input);
  /*if (user_friendly(input)){
    std::cout<<"Invalid input. Please retry with the format: <letter a-f> <number 0-12>"<<std::endl<<"Points A2, A5, A11, C1, C2, C4, C5, C6, C7, C8, C10, C11, E2, E5, and E11 do not exist."<<std::endl;
    return input;
  }
  else {*/
    for (int i = min_x; i <= max_x; i++) {
      if (i == x_) {
        input.metric_x = (i*grid_dist);
      }
    }

    for (int n = min_y; n <= max_y; n++) {
      if (n == y_) {
        input.metric_y = (max_y_dist-(n-min_y)*grid_dist);
      }

    }
    return input.is_valid;
  //}
}

int main(int argc, char const *argv[]){
  char y;
  int x;
  std::cout<<"Input field coordinates"<<std::endl;
  std::cin>>y;
  std::cin>>x;
  point_1.y_coord = toupper(y);
  point_1.x_coord = x;
  convert_to_metric(point_1);
  std::cout<<point_1.metric_x<<" cm, "<<point_1.metric_y<<" cm"<<std::endl;
}
