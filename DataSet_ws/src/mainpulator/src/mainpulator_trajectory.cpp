#include "mainpulator/mainpulator_trajectory.h"
using namespace std;
namespace trajectory{

    vector<double> p_to_p(double time)
    {
        double q = 0;
        double qd = 0;
        double qdd = 0;
        double t = time ;
        double at = 0.5;
        if (t < 0.5)
        {
                /*qdd = 1;
                qd = t*qdd;
                q = 0.5*qdd*t*t;*/
                /*	qdd = 2;
                    qd = t*qdd;
                    q = 0.5*qdd*t*t;*/
            qdd = at;
            qd = t * qdd;
            q = 0.5 * qdd * t * t;        
        }
        else if (t >= 0.5 && t < 4.5)
        {
                /*qdd = 0;
                qd = 0.5;
                q = 0.125+qd*(t-0.5);*/

                /*qdd = 0;
                qd = 1;
                q = 0.25+qd*(t-0.5);*/

            qdd = 0;
            qd = at * 0.5;
            q = 0.5 * at * 0.5 * 0.5 + qd * (t - 0.5);



        }
        else if (t >= 4.5 && t < 5)
        {
                /*9qdd = -1;
                qd = 0.5+qdd*(t-4.5);
                q = 2.125 + 0.5*(t-4.5)+0.5*qdd*(t-4.5)*(t-4.5);*/
                /*qdd = -2;
                qd = 1+qdd*(t-4.5);
                q = 4.25 + 1*(t-4.5)+0.5*qdd*(t-4.5)*(t-4.5);*/

            qdd = -at;
            qd = at * 0.5 + qdd * (t - 4.5);
            q = 0.5 * at * 0.5 * 0.5 + at * 0.5 * 4 + at * 0.5 * (t - 4.5) + 0.5 * qdd * (t - 4.5) * (t - 4.5);
        }
        else if (t >= 5 && t < 10)
        {
                /*qdd = 0;
                qd = 0;
                q = 2.25;*/


                /*qdd = 0;
                qd = 0;
                q = 4.5;*/

            qdd = 0;
            qd = 0;
            q = 0.5 * at * 0.5 * 0.5 + at * 0.5 * 4 + at * 0.5 * 0.5 - 0.5 * at * 0.5 * 0.5;
        }
        else if (t >= 10 && t < 10.5)
        {


                /*qdd = -1;
                qd = qdd*(t-10);
                q = 2.25+0.5*qdd*(t-10)*(t-10);*/

                /*	qdd = -2;
                    qd = qdd*(t-10);
                    q = 4.5+0.5*qdd*(t-10)*(t-10);*/


            qdd = -at;
            qd = qdd * (t - 10);
            q = 0.5 * at * 0.5 * 0.5 + at * 0.5 * 4 + at * 0.5 * 0.25 + 0.5 * qdd * (t - 10) * (t - 10);

        }
        else if (t >= 10.5 && t < 14.5)
        {
                /*qdd = 0;
                qd = -0.5;
                q = 2.125+qd*(t-10.5);*/


                /*qdd = 0;
                qd = -1;
                q = 4.25+qd*(t-10.5);*/
            qdd = 0;
            qd = -at * 0.5;
            q = 0.5 * at * 0.5 * 0.5 + at * 0.5 * 4 - at * 0.5 * (t - 10.5);


        }
        else if (t >= 14.5 && t < 15)
        {
                /*qdd = 1;
                qd = -0.5+qdd*(t-14.5);
                q = 0.125 - 0.5*(t-14.5)+0.5*qdd*(t-14.5)*(t-14.5);
                */

                /*qdd = 2;
                qd = -1+qdd*(t-14.5);
                q = 0.25 - (t-14.5)+0.5*qdd*(t-14.5)*(t-14.5); */


            qdd = at;
            qd = -at * 0.5 + qdd * (t - 14.5);
            q = 0.5 * at * 0.5 * 0.5 - at * 0.5 * (t - 14.5) + 0.5 * qdd * (t - 14.5) * (t - 14.5);
        }
        else if (t >= 15 && t < 20)
        {
            qdd = 0;
            qd = 0;
            q = 0;
        }
        else if(t>=20)
        {
            t = 0;
            qdd = 0;
            qd = 0;
            q = 0;
        }
       //q=q-0.5;
        return {q,qd,qdd};

    }

}