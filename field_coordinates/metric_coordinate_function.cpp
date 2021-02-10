#include <iostream>

//checks input against nonexistant points on the grid
bool user_friendly(char x, double y) {
  double a_and_e[] = {2, 5, 11};
  double row_c[] = {1, 2, 4, 5, 6, 7, 8, 10, 11};
  bool answer = false;
  //checks for A2, A5, A11, E2, E5, and E11
  for (int i = 0; i<3; i++) {
    if ((x == 'A'||x == 'E') && (y == a_and_e[i])) {
      answer = true;
    }
  }
  //Checks for C1, C2, C4, C5, C6, C7, C8, C10, and C11
  for (int i = 0; i<9; i++) {
    if (x == 'C' && y == row_c[i]) {
      answer = true;
    }
  }
 return answer;
}
void metric_coords(char x,double y){
  double coords[2] = {0,0};
  double grid_dist = 76.2;
  double max_y_dist = 381;
  //Capitalizes first character with ASCII
  if (x>70){
    x-=32;
  }
  //Translates letter into y-coordinate (vertical)
  for (int i = 'A'; i < 'F'; i++){
    if (i == x) {
      coords[1] = (max_y_dist-(i-'A')*grid_dist);
    }
  }
  //Translates number into x-coordinate (horizontal)
  for (int i = 1; i < 12;i++){
    if (i == y) {
      coords[0] = (i*grid_dist);
    }
  }
  //Validates input, effectively checks for anything not an a-e or 1-11
  if (((coords[0] == 0 ) || (coords[1] == 0)) || user_friendly(x,y))
  {
    //Error message
    std::cout<<"Invalid input. Please retry with the format: <letter a-e> <number 1-11>"<<std::endl<<"Points A2, A5, A11, C1, C2, C4, C5, C6, C7, C8, C10, C11, E2, E5, and E11 do not exist."<<std::endl;
  }
  else
  {
  std::cout<<coords[0]<<" cm, "<<coords[1]<<" cm"<<std::endl;
  }
}

int main(int argc, char const *argv[]){
  //Plugs user input into metric_coords 
  char a;
  double b;
  std::cout<<"Input field coordinates"<<std::endl;
  std::cin>>a;
  std::cin>>b;
  metric_coords (a,b);
  return 0;
}
