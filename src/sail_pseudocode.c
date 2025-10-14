#include <math.h>
#include <stdbool.h>
struct vector{
  float x_comp;
  float y_comp;
  float z_comp;
}
int dot(struct vector v1; struct vector v2){
  return v1.x_comp*v2.x_comp+v1.y_comp*v2.y_comp+v1.z_comp*v2.z_comp
}
struct vector cross(struct vector v1, struct vector v2){
  struct vector output;
  output.z_comp = v1.x_comp * v2.y_comp - v2.x_comp * v1.y_comp;
  output.x_comp = v1.y_comp * v2.z_comp - v2.y_comp * v1.z_comp;
  output.y_comp = v1.z_comp * v2.x_comp - v2.z_comp * v1.x_comp;
}
int magnitude(struct vector v1){
  return sqrt(v1.x_comp*v1.x_comp+v1.y_comp*v1.y_comp+v1.y_comp*v1.y_comp);
}
float deg(float anglerad){
  return 180/M_PI * anglerad;
}
float rad(float angledeg){
  return M_PI/180*angledeg;
}
bool isdead(struct vector vel, struct vector appwind, int angle){//Is it in the cone of death?
  float cosine = dot(vel,appwind)/(magnitude(vel)*magnitude(appwind));
  if (cosine>cos(angle)){
    return true;
  }
  else{
    return false;
  }
}
int angle(struct vector v1,struct vector v2){
  return arccos(dot(v1,v2)/(magnitude(v1)*magnitude(v2)));
}
