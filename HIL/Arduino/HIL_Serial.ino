// Author: Waldenê Moura
//

#include <math.h>     
#include "HIL_Variables.h"
#include "HIL_Waypoints.h"     // Altere este arquivo de acordo com os way points a serem alcançados

void setup() {
  // Inicializa porta serial
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  // Inicializa variáveis
  fncInicializa();
}

void loop() {

  // Tem informaçao na porta serial
  if (Serial.available() >= 13) {

    if (Serial.read() == 'W') {

      // Recebe as variaveis Y de entrada pela ordem enviada
      fncRecebe_Y();

      // Pisca Led para informar leitura e funcionamento
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

      // Atualização dos Estados discretos
      float _pos_r_1 = _WP[0][_idx + 1] - _Y.Xn.number;
      float _pos_r_2 = _WP[1][_idx + 1] - _Y.Yn.number;
      
      if (_phases == 1) {

        // O ângulo de guinada de referência,
        _psiR = LineOfSight(_pos_r_2, _pos_r_1, _Y.psi.number);         // psiR = LineOfSight(pos_r(2), pos_r(1), u(9));
        if ( rad2deg(fabs(_Y.psi.number - _psiR)) <= 5.0) {             // if rad2deg(abs(u(9) - psiR)) <= 5.0
          _phases = 2;                                                //     phases = phases + 1;
        }                                                             // end
        
      } else if(_phases == 2) {

        if (_WP[0][_idx+1] - _WP[0][_idx] != 0.0) {
            float m  = (_WP[1][_idx+1] - _WP[1][_idx]) / (_WP[0][_idx+1] - _WP[0][_idx]);                             // Eq (5.4)
            float er = fabs( (_Y.Yn.number - m*_Y.Xn.number - (_WP[1][_idx] - m*_WP[0][_idx]) ) / (sqrt(m*m+1.0)) );  // Eq (5.3) sqrt funciona para double e float
            Nr =  _Y.Xn.number - er*_Y.psi.number;          // Eq (5.7)
            Er =  _Y.Yn.number - er*_Y.psi.number;          // Eq (5.8)

            // P   Matriz           Vetor
            // 2	  max(svd(X))	   sum(abs(X).^2)^(1/2)
            d = sqrt(pow(_WP[0][_idx+1] - Nr, 2.0) + pow(_WP[1][_idx+1] - Er, 2.0));                    // Eq (5.6) d = norm(_WP[1:2,idx+1])-([Nr; Er]), 2);
            float ds = sqrt(pow(_WP[0][_idx+1]-_WP[0][_idx], 2.0) + pow(_WP[1][_idx+1]-_WP[1][_idx], 2.0)); // ds = norm(_WP[1:2,idx+1])-_WP[1:2,idx], 2);
            float eMax = d*(70.0+40.0)*0.5/ds;                                                        // Eq (5.5)

            if (er > eMax) {
                _psiR = LineOfSight(_pos_r_2, _pos_r_1, _Y.psi.number);
            } 
        } else {
            _psiR = LineOfSight(_pos_r_2, _pos_r_1, _Y.psi.number);
            d = sqrt(pow(_pos_r_1,2) + pow(_pos_r_2,2));      // d = norm(pos_r, 2);
        }
        
        if (d <= _lambda) {
            _idx = _idx + 1;
            _phases = 1;                
        }

      }

      // Fim de simulação
      if (_idx == _indiceWP-1) {
        _stop = 1.0;
      }

      // sys=mdlOutputs(x,u,WP) Função saída do S-Function
      // Cálculo da saída
      if (_phases == 2) {
        _UU.Vt_r = _WP[3][_idx + 1];           // Próximo VT_r
        _UU.H_r = _WP[2][_idx + 1];           //         h_r
      } else {
        _UU.Vt_r = _WP[3][_idx];               // Atual VT_r
        _UU.H_r = _WP[2][_idx];               //       h_r
      }
      _UU.psi_r = _psiR;

      // Controle Longitudinal
      // Throtle
      Throtle_E_n_1 = -_Y.Vt.number * Kvt + _UU.Vt_r; // -Vt * Kvt + Vt_r
      Throtle_U_n_1 = Throtle_U_n + 0.03041 * Throtle_E_n_1 - 0.03019 * Throtle_E_n;
      if (Throtle_U_n_1 > 1) { // Saturação
        Throtle_U_n_1 = 1;
      } else if (Throtle_U_n_1 < -0.0416) {
        Throtle_U_n_1 = -0.0416;
      }
      Throtle_U_n = Throtle_U_n_1;
      Throtle_E_n = Throtle_E_n_1;

      // Controle Látero Direcional
      // Aileron
      Aileron_E_n_1 =
          -_Y.phi.number +
          0.2716 * (_UU.psi_r -
                    _Y.psi.number); // -theta * Kphi + Kpsi * (psi_r - psi)
      Aileron_U_n_1 =
          Aileron_U_n - 0.0908 * Aileron_E_n_1 + 0.0906 * Aileron_E_n;
      Aileron_U_n = Aileron_U_n_1;
      Aileron_E_n = Aileron_E_n_1;
      
      // Rudder
      Rudder_E_n_1 = _Y.r.number; // r
      Rudder_U_n_1 = 0.99 * Rudder_U_n + 0.1045 * (-Rudder_E_n_1 + Rudder_E_n);
      Rudder_U_n = Rudder_U_n_1;
      Rudder_E_n = Rudder_E_n_1;

      // Pitch Atitude Hold
      _u_add_y = _UU.H_r - _Y.H.number;
      _temp1 = (_u_add_y > 0.0 ? 0.0873 : -0.0873);

      if (fabs(_u_add_y) > 5.0) {
        _tempTheta = _Y.theta.number;
        _tempQ = _Y.q.number;
      } else {
        _temp1 = 0.0;
        _tempTheta = 0.0;
        _tempQ = 0.0;
      }

      PitchAltitude_E_n_1 = _temp1 - _tempTheta * Ktheta;
      PitchAltitude_U_n_1 = PitchAltitude_U_n - 0.4565 * PitchAltitude_E_n_1 +
                            0.4499 * PitchAltitude_E_n;
      PitchAltitude_U_n = PitchAltitude_U_n_1;
      PitchAltitude_E_n = PitchAltitude_E_n_1;

      // Pitch Atitude Hold
      _u_add_y = _UU.H_r - _Y.H.number;

      if (fabs(_u_add_y) > 5.0) {
        _temp2 = 0;
        _tempH = 0;
        _tempTheta1 = 0;
        _tempQ1 = 0;
      } else {
        _temp2 = _UU.H_r;
        _tempH = _Y.H.number;
        _tempTheta1 = _Y.theta.number;
        _tempQ1 = _Y.q.number;
      }

      Altitude_E_n_1 = _temp2 - _tempH * Kh;
      if (Altitude_E_n_1 > 5.0) {
        Altitude_E_n_1 = 5.0;
      } else if (Altitude_E_n_1 < -5.0) {
        Altitude_E_n_1 = -5.0;
      }

      Altitude_U_n_1 =
          Altitude_U_n + 0.01723 * Altitude_E_n_1 - 0.01717 * Altitude_E_n;
      Altitude_U_n = Altitude_U_n_1;
      Altitude_E_n = Altitude_E_n_1;

      sAltitude_E_n_1 = Altitude_U_n_1 - _tempTheta1 * Ktheta;
      sAltitude_U_n_1 =
          sAltitude_U_n - 0.4565 * sAltitude_E_n_1 + 0.4499 * sAltitude_E_n;
      sAltitude_U_n = sAltitude_U_n_1;
      sAltitude_E_n = sAltitude_E_n_1;

      //////////////////////  Envio dos parâmetros

      // Envia header
      Serial.write('A');

      // Gerando du
      // Envia Throtle
      temp.number = Throtle_U_n_1;
      fncEnvia(temp);

      // Pitch Atitude Hold + Altitude Hold
      temp.number =
          PitchAltitude_U_n_1 - _tempQ * Kq + sAltitude_U_n_1 - _tempQ1 * Kq;
      fncEnvia(temp);

      // Envia Aileron
      temp.number = Aileron_U_n_1 - _Y.p.number * 0.119;
      fncEnvia(temp);

      // Envia Rudder
      temp.number = Rudder_U_n_1;
      fncEnvia(temp);

      // Envia sinal de parada
      temp.number = _stop;
      fncEnvia(temp);

      // Envia H de referência
      temp.number = _UU.H_r;
      fncEnvia(temp);

      // Envia psi de referência
      temp.number = _UU.psi_r;
      fncEnvia(temp);

      // Envia terminator
      Serial.write(10);

      delay(80);      // 80, 80 for (0.01)
    }
  }

  // Sincroniza
  delay(40);
}

// LineOfSight function
float LineOfSight(float y, float x, float psi) {
    float LOS = atan2(y, x);        

    if (fabs(LOS - psi) > PI) {
        float psiR = LOS + 2*PI*truncf(psi/(2*PI));
        if (fabs(psiR - psi) > PI) {
            float aux = psiR - 2*PI; 
            if (fabs(aux - psi) < PI) {
                LOS = aux;
            } else {
                aux = psiR + 2*PI;
                if (fabs(aux - psi) < PI) {
                    LOS = aux;
                }
            }
        } else {
            LOS = psiR;
        }
    }

    return(LOS);
}

// Recebe as variaveis Y 
void fncRecebe_Y() {
  _Y.Vt.number = fncTraz();     // Vt
  _Y.alpha.number = fncTraz();  // alpha
  _Y.beta.number = fncTraz();   // beta
  _Y.p.number = fncTraz();      // p
  _Y.q.number = fncTraz();      // q
  _Y.r.number = fncTraz();      // r
  _Y.phi.number = fncTraz();    // phi
  _Y.theta.number = fncTraz();  // theta
  _Y.psi.number = fncTraz();    // psi
  _Y.Xn.number = fncTraz();     // Xn
  _Y.Yn.number = fncTraz();     // Yn
  _Y.H.number = fncTraz();      // H
}

// Traz informação da Serial
float fncTraz() {
  var_union temp;
  
  for (int i = 0; i < 4; i++) {
    temp.bytes[i] = Serial.read();
  }

  return temp.number;
}

// Envia informação pela Serial
void fncEnvia(var_union x) {
  for (int i = 0; i < 4; i++) {
    Serial.write(x.bytes[i]);
  }
}

// Inicializa Variáveis
void fncInicializa() {
  
  // Inicializando Látero-Direcional
  // Controle Rudder
  Rudder_U_n = 0;
  Rudder_U_n_1 = 0;
  Rudder_E_n = 0;
  Rudder_E_n_1 = 0;
  
  // Controle Aileron
  Aileron_U_n = 0;
  Aileron_U_n_1 = 0;
  Aileron_E_n = 0;
  Aileron_E_n_1 = 0;

  // Inicializando Longitudinal
  // Controle Throtle
  Throtle_U_n = 0;
  Throtle_U_n_1 = 0;
  Throtle_E_n = 0;
  Throtle_E_n_1 = 0;

  // Pitch Atitude Hold
  _u_add_y = 0;
  _temp1 = 0;
  _tempTheta = 0;
  _tempQ = 0;
  PitchAltitude_U_n = 0;
  PitchAltitude_U_n_1 = 0;
  PitchAltitude_E_n = 0;
  PitchAltitude_E_n_1 = 0;

  // Altitude hold
  _temp2 = 0;
  _tempH = 0;
  _tempTheta1 = 0;
  _tempQ1 = 0;
  Altitude_U_n = 0;
  Altitude_U_n_1 = 0;
  Altitude_E_n = 0;
  Altitude_E_n_1 = 0;
  sAltitude_U_n = 0;
  sAltitude_U_n_1 = 0;
  sAltitude_E_n = 0;
  sAltitude_E_n_1 = 0;

  // Inicialização da S-Function Guidance
  _stop = 0.;
  _lambda = 10.0;
  _idx = 0;        // Corresponde ao x0 = [1 0 1]  idx inicializa do zero
  _psiR = 0.;
  _phases = 1;

  // Valores iniciais
  _UU.Vt_r = _WP[3][_idx];               // Atual VT_r
  _UU.H_r = _WP[2][_idx];               //       h_r
  _UU.psi_r = _psiR;
  d = 0.0;
  Nr = 0.0;
  Er = 0.0;
}
