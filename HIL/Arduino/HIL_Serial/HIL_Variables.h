// Definição das variáveis utilizadas

#define PI 3.14159265
#define rad2deg(x) (x*180.0/PI)

// Uniao para facilitar o manuseio dos bytes dentro das variaveis
typedef union {
  float number;
  uint8_t bytes[4];
} var_union;

// Variaveis de controle
var_union temp;

typedef struct // Cria uma STRUCT para armazenar os dados de entrada Y (HIL)
{
  var_union Vt, alpha, beta,    p,    q,    r,   phi, theta,  psi,    Xn,    Yn,     H;
} Vector_Y;

typedef struct // Cria uma STRUCT para armazenar os dados de entrada U (Guiagem)
{
  float Vt_r, H_r, psi_r;
} Vector_U;

Vector_Y _Y; 
Vector_U _UU;

// Variáveis Controle Látero-Direcional
float Rudder_U_n, Rudder_U_n_1, Rudder_E_n, Rudder_E_n_1;           // Rudder
float Aileron_U_n, Aileron_U_n_1, Aileron_E_n, Aileron_E_n_1;       // Aileron

// Variáveis Controle Longitudinal
float Throtle_U_n, Throtle_U_n_1, Throtle_E_n, Throtle_E_n_1;       // Throtle
#define Kvt 1.0

float _u_add_y, _temp1, _tempTheta, _tempQ;                         // Pitch Altitude Hold 
float PitchAltitude_U_n, PitchAltitude_U_n_1, PitchAltitude_E_n, PitchAltitude_E_n_1;
#define Ktheta 1.0
#define Kq 0.0877

// Altitude
float _temp2, _tempH, _tempTheta1, _tempQ1;
float Altitude_U_n, Altitude_U_n_1, Altitude_E_n, Altitude_E_n_1;
float sAltitude_U_n, sAltitude_U_n_1, sAltitude_E_n, sAltitude_E_n_1;
#define Kh 1.0

// Guidance
float _stop, _psiR, _lambda;
int _idx, _phases;

float d;           // Eq (5.6)
float Nr;          // Eq (5.7)
float Er;          // Eq (5.8)
