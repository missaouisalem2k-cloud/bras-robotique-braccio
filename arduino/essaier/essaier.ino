#include <Braccio.h>
#include <servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Structure pour stocker une position (angles des 6 servos)
struct Position {
  int base;
  int shoulder;
  int elbow;
  int wrist_rot;
  int wrist_ver;
  int gripper;
};
Position trajectoire[] = {
 {180, 45, 180, 180, 90, 0},
  {180, 45, 180, 180, 90, 0},
  {180, 90, 180, 180, 90, 60},
  {0, 45, 180, 180, 45, 60},
  {0, 90, 180, 180, 90, 0},
  {0, 45, 180, 45, 0, 0}
};
int nbPositions = sizeof(trajectoire) / sizeof(trajectoire[0]);
void setup () {
  Braccio.begin();
}

void executerTrajectoire(Position traj[], int taille, int delayStep) {
  for (int i = 0; i < taille; i++) {
    Braccio.ServoMovement(delayStep,
      traj[i].base,
      traj[i].shoulder,
      traj[i].elbow,
      traj[i].wrist_rot,
      traj[i].wrist_ver,
      traj[i].gripper
    );
    delay(1000); // pause entre chaque mouvement, tu peux ajuster
  }
}

void loop () {
  executerTrajectoire(trajectoire, nbPositions, 20);
  delay(2000); // pause avant de recommencer la trajectoire
}
