#include"turtlelib/se2d.hpp"
#include<turtlelib/geometry2d.hpp>
#include<iostream>
#include"turtlelib/svg.hpp"
#include<sstream>
#include<cmath>


using namespace turtlelib;
using namespace std;
using turtlelib::Transform2D;

int main(){
    Transform2D Tab, Tbc, Tac , Tba, Tcb, Tca;
    SVG svg;
    



    cout<<"Enter transform T_{a,b}:\n";
    cin>>Tab;
    cout<<"Enter transform T_{b,c}:\n";
    cin>>Tbc;
    Tac = Tab*Tbc;
    Tba = Tab.inv();
    Tcb = Tbc.inv();
    Tca = Tac.inv();
    Transform2D origin = Transform2D(Vector2D{0,0}, 0);
    svg.draw_Transform2D(origin, "a");
    svg.draw_Transform2D(Tab, "b");
    svg.draw_Transform2D(Tac, "c");

    cout<<"T_{a,b}: "<<Tab<<endl;
    cout<<"T_{b,a}: "<<Tba<<endl;
    cout<<"T_{b,c}: "<<Tbc<<endl;
    cout<<"T_{c,b}: "<<Tcb<<endl;
    cout<<"T_{a,c}: "<<Tac<<endl;
    cout<<"T_{c,a}: "<<Tca<<endl;


    Point2D Pa, Pb, Pc;
    cout<<"Enter point p_a:\n";
    cin>>Pa;
    Pb = Tba(Pa);
    Pc = Tca(Pa);
    svg.draw_point(Pa, origin,  "purple");
    svg.draw_point(Pb, Tab ,"brown");
    svg.draw_point(Pc, Tac, "orange");

    cout<<"p_a: "<<Pa<<endl;
    cout<<"p_b: "<<Pb<<endl;
    cout<<"p_c: "<<Pc<<endl;

    Vector2D va, vb, vc, vbhat;
    cout<<"Enter vector v_b:\n";
    cin>>vb;
    vbhat = normalize_vector(vb);
    va = Tab(vb);
    vc = Tcb(vb);
    svg.draw_line(va, Vector2D{0,0}, origin, "purple");
    svg.draw_line(vbhat, Vector2D{0,0}, Tab, "brown");
    svg.draw_line(vc,  Vector2D{0,0}, Tac, "orange");


    vbhat = normalize_vector(vb);
    cout<<"v_bhat: "<<vbhat<<endl;
    cout<<"v_a: "<<va<<endl;
    cout<<"v_b: "<<vb<<endl;
    cout<<"v_c: "<<vc<<endl;

    Twist2D Va, Vb, Vc;
    cout<<"Enter twist V_b:\n";
    cin>>Vb;
    Va = Tab(Vb);
    Vc = Tcb(Vb);
    cout<<"V_a: "<<Va<<endl;
    cout<<"V_b: "<<Vb<<endl;
    cout<<"V_c: "<<Vc<<endl;

    svg.export_svg("/tmp/frames.svg");

}