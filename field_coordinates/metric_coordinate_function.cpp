#include <iostream>

bool user_friendly(char x, double y) {
  double a_and_e[] = {2, 5, 11};
  double row_c[] = {1, 2, 4, 5, 6, 7, 8, 10, 11};
  bool answer = false;
  for (int i = 0; i<3; i++) {
    if ((x == 65||x == 69) && (y == a_and_e[i])) {
      answer = true;
    }
  }
  for (int i = 0; i<9; i++) {
    if (x == 67 && y == row_c[i]) {
      answer = true;
    }
  }
 return answer;
}
void metric_coords(char x,double y){
  double coords[2] = {0,0};
  if (x>70){
    x-=32;
  }
  for (int i = 65; i < 70; i++){
    if (i == x) {
      coords[1] = (150-((i-65)*30))*2.54;
    }
  }
  for (int i = 1; i < 12;i++){
    if (i == y) {
      coords[0] = (i*76.2);
    }
  }
  if (((coords[0] == 0 ) or (coords[1] == 0)) || user_friendly(x,y))
  {
    std::cout<<"Invalid input. Please retry with the format: <letter a-e> <number 1-11>"<<std::endl<<"Points A2, A5, A11, C1, C2, C4, C5, C6, C7, C8, C10, C11, E2, E5, and E11 are not accepted."<<std::endl;
  }
  else
  {
  std::cout<<coords[0]<<" cm, "<<coords[1]<<" cm"<<std::endl;
  }
}

int main(int argc, char const *argv[]){
  char a;
  double b;
  std::cout<<"Input field coordinates"<<std::endl;
  std::cin>>a;
  std::cin>>b;
  metric_coords (a,b);
  return 0;
}
