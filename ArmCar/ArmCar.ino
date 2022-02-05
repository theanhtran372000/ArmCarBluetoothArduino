/* -------------------    Thư viện   -------------------- */
#include<Servo.h>
#include<SoftwareSerial.h>

/* ------------------- Khai báo chân -------------------- */
// Cánh tay robot
#define bantayPin A3  // Pin của servo bàn tay
#define khuytayPin A1 // Pin của servo khủy tay
#define vaiPin A2     // Pin của servo vai
#define hongPin A0    // Pin của servo hông

// Chân điều khiển động cơ DC
const int ena = 6;    // Chân điều khiển tốc độ bánh trái (chân băm xung)
const int in1 = 7;    // Chân input 1 điều khiển bánh trái
const int in2 = 5;    // Chân input 2 điều khiển bánh trái
const int enb = 3;    // Chân điều khiển tốc độ bánh phải (chân băm xung)
const int in3 = 4;    // Chân input 3 điều khiển bánh phải
const int in4 = 2;    // Chân input 4 điều khiển bánh phải

// Chân cảm biến siêu âm đo khoảng cách
const int echo = 12;  // Chân đo thời gian
const int trig = 13;  // Chân phát sóng

/* ------------------- Khai báo biến + hằng ------------------- */
// Hằng
const int MAX_RECORDING_LENGTH = 20; // Độ dài tối đa của Record
const int armDelay = 10 ;   // Thời gian delay giữa 2 lần điều khiển
const int avoidDelay = 10;  // Thời gian delay giữa 2 lần đo khoảng cách
const int minDistance = 30; // 20cm là khoảng cách tối thiểu để tránh vật cản
const int maxDistance = 300;// Khoảng cách tối đa của hàm đo khoảng cách
                            // Mặc dù khoảng cách tối đa của cảm biến sóng âm là 450 cm
                            // Nhưng em sẽ chỉ giới hạn ở mức 300cm để kết quả được chính xác hơn

// Biến
// 4 Servo điều khiển cánh tay robot cùng với phạm vi quay tối đa của mỗi servo
Servo bantayServo;
int bantayMin, bantayMax;
int gocBantay;

Servo khuytayServo;
int khuytayMin, khuytayMax;
int gocKhuytay;

Servo vaiServo;
int vaiMin, vaiMax;
int gocVai;

Servo hongServo;
int hongMin, hongMax;
int gocHong;

// Biến record chuyển động tay
int isRecordingArm = 0;
int dsChuyenDongTay[20]; // 1 là bàn tay, 2 là khủy tay, 3 là vai, 4 là hông
int gocBatDau[MAX_RECORDING_LENGTH];
int gocKetThuc[MAX_RECORDING_LENGTH];
int soChuyenDongTay = 0;

// Biến khác
int i, j;

// Bluetooth
SoftwareSerial Bluetooth(0, 1); // Khai báo RX = 0, TX = 1, kết nối tới HC06
int dataIn;                     // Dữ liệu truyền tới

/* --------------- Khai báo hàm tự định nghĩa ----------------- */

/* ---------------     Hàm điều khiển xe      ----------------- */
// Chuyển động từng cặp bánh xe
// Hai bánh bên trái tiến
void tienTrai(int tocdo){
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(ena, tocdo);
}

// Hai bánh bên trái lùi
void luiTrai(int tocdo){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ena, tocdo);
}

// Dừng 2 bánh bên trái
void dungTrai(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ena, 0);
}

// Hai bánh bên phải tiến
void tienPhai(int tocdo){
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enb, tocdo);
}

// Hai bánh bên phải lùi
void luiPhai(int tocdo){
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enb, tocdo);
}

// Dừng 2 bánh bên phải
void dungPhai(){
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(ena, 0);
}


// Chuyển động tổng thể của xe

// Xe tiến lên
void tienXe(int tocdo){
  tienTrai(tocdo);
  tienPhai(tocdo);
}

// Xe lùi
void luiXe(int tocdo){
  luiTrai(tocdo);
  luiPhai(tocdo);
}

// Xe dừng lại
void dungXe(){
  dungTrai();
  dungPhai();
}

// Quay phải: phải lùi, trái tiến cùng tốc độ
void rePhai(int tocdo){
  luiPhai(tocdo);
  tienTrai(tocdo);
}

// Quay trái: lùi trái, tiến phải cùng tốc độ
void reTrai(int tocdo){
  luiTrai(tocdo);
  tienPhai(tocdo);
}

/* --------------- Hàm tránh vật cản ----------------- */
// Đưa cánh tay về vị trí ban đầu
void resetArm(){
  gocBantay = 40;
  bantayServo.write(gocBantay);

  gocKhuytay = 10;
  khuytayServo.write(gocKhuytay);

  gocVai = 0;
  vaiServo.write(gocVai);

  gocHong = 90;
  hongServo.write(gocHong);
}

// Hàm xác định khoảng cách bằng cảm biến siêu âm
// Đơn vị là centimet (cm)
int doKhoangCach(){
  // Phát xung bằng cách truyền vào chân trig một xung cao độ rộng tối thiểu 10 ms
  digitalWrite(trig, LOW);    // Đầu tiên đưa chân trig xuống mức thấp
  delayMicroseconds(2);       // delay 1 chút để chân trig xuống mức thấp
  digitalWrite(trig, HIGH);   // Đưa chân trig lên mức cao và giữ trong 10ms
  delayMicroseconds(10);
  digitalWrite(trig, LOW);    // Đưa lại chân trig về LOW

  // Ngay khi phát xung siêu âm đi thì lập tức chân echo sẽ được đặt lên mức cao
  // Khi nhận được tín hiệu trả về, nó sẽ được đưa trở lại mức thấp
  // Ta đo thời gian chân echo ở mức cao và tính ra khoảng cách
  unsigned long thoigian = pulseIn(echo, HIGH); // microseconds
  Serial.println(thoigian);

  // Tốc độ âm thanh trong không khí là 340m/s 
  // Tương đương với 29.412 microsecs để đi 1 cm
  // Sóng âm phải phát đi và dội lại nên khoảng cách phải chia 2
  int khoangcach = int (thoigian / 2 / 29.412);
  
  return min(khoangcach, maxDistance); // giới hạn kết quả trả về (tối đa 300cm = 3m)
}

// Hàm tránh vật cảm
void tranhVatCan(){  
  // Đo khoảng cách
  long khoangCachGiua = 0;
  long khoangCachPhai = 0;
  long khoangCachTrai = 0;

  do{
    // Đo khoảng cách phía trước
    khoangCachGiua = doKhoangCach();
    delay(avoidDelay);

    // Nếu khoảng cách tới vật cản nhỏ hơn 30cm (có thể đổi số khác)
    if (khoangCachGiua <= minDistance){
      // Lập tức dừng xe lại
      dungXe();
      delay(100); // delay để xe ổn định vị trí

      // Quay xe sang phải để tìm đường (khoảng 90 độ)
      rePhai(200);
      delay(500);
      dungXe();
      khoangCachPhai = doKhoangCach();
      delay(500);

      // Quay xe sang trái để tìm đường (khoảng 180 độ)
      reTrai(200);
      delay(1000); //  gấp đôi rẽ phải do đã rẽ phải trước đó
      dungXe();
      khoangCachTrai = doKhoangCach();
      delay(500);

      // Kiểm tra khoảng cách
      // Một trong 2 bên trống
      if(khoangCachTrai > minDistance || khoangCachPhai > minDistance){
        // Bên trái đường rộng rãi hơn
        if(khoangCachTrai > khoangCachPhai){
          // Không làm gì do đã đi đúng hướng
        }

        // Ngược lại
        else{
          // Đang rẽ trái nên quay ngược xe lại
          rePhai(200);
          delay(1000);
          dungXe();
          delay(500);
        }
      }
      
      // Nếu cả 2 bên đều không có đường thì quay xe
      else{
        rePhai(200);
        delay(500);
        dungXe();
        delay(500);
      }
    }
    else{
      tienXe(200);
    }

    // Kiểm tra tín hiệu Bluetooth
    if(Bluetooth.available() > 0){
      dataIn = Bluetooth.read();
    }
  }while(dataIn == 13);
}

/* ------------------- Hàm record -------------------- */


/* ------------------- Hàm cài đặt ban đầu -------------------- */
void setup() { 
  // Khởi tạo kết nối Bluetooth
  Bluetooth.begin(9600);
  Bluetooth.setTimeout(5); // 5ms
  delay(20);

  // Khởi tạo tốc độ truyền cho Serial
  Serial.begin(9600);

  // Khai báo chân input, output
  // Chân điều khiển động cơ
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(enb, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Chân cảm biến sóng âm
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Thiết lập servo
  // Phần hông quay
  hongServo.attach(hongPin, 450, 2500); // mở rộng góc quay (0 -> 180) tính ra được thời gian ms: 450 -> 2500ms
  hongMin = 0;
  hongMax = 180;

  // Phần vai
  vaiServo.attach(vaiPin);
  vaiMin = 0;
  vaiMax = 60;
  
  // Phần khủy tay
  khuytayServo.attach(khuytayPin);
  khuytayMin = 0;
  khuytayMax = 60;

  // Phần bàn tay
  bantayServo.attach(bantayPin);
  bantayMin = 20;
  bantayMax = 40;

  // Reset cánh tay về vị trí đầu
  resetArm();
} 

/* -------------------      Hàm lặp       -------------------- */
void loop() { 
  // Kiểm tra tín hiệu từ app
  if(Bluetooth.available() > 0){
    // Nhận tín hiệu
    dataIn = Bluetooth.read();
    Serial.print("Tín hiệu: ");
    Serial.println(dataIn);

    /* ------------------- Di chuyển xe theo yêu cầu -------------------- */

    // Xe tiến
    if(dataIn == 0){
      tienXe(200);
    }

    // Rẽ trái
    if(dataIn == 1){
      reTrai(150);
    }

    // Rẽ phải
    if(dataIn == 2){
      rePhai(150);
    }
    
    // Lùi xe
    if(dataIn == 3){
      luiXe(200);
    }

    // Dừng xe
    if(dataIn == 4){
      dungXe();
    }

    /* ------------------- Di chuyển cánh tay -------------------- */
    // Việc điều khiển cánh tay robot sẽ tăng hoặc giảm liên tục góc của bộ phận đang điều khiển
    // Cho tới khi nhận được tín hiệu điều khiển khác hoặc tín hiệu dừng (99)
    // App sẽ gửi tín hiệu dừng khi người dùng nhả phím
     
    // Tăng góc bàn tay 
    if(dataIn == 5){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 1; // Chuyển động bàn tay
          gocBatDau[soChuyenDongTay] = gocBantay;
        }
      }

      while(dataIn == 5){
        // Tăng góc
        gocBantay ++;
        gocBantay = constrain(gocBantay, bantayMin, bantayMax);
        bantayServo.write(gocBantay);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();
  
          // Kết thúc chuyển động
          if(dataIn != 5 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocBantay;
            soChuyenDongTay ++;
          }
        }
      }
    }
    

    // Giảm góc bàn tay
    if(dataIn == 6){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 1; // Chuyển động bàn tay
          gocBatDau[soChuyenDongTay] = gocBantay;
        }
      }

      while(dataIn == 6){
        // Giảm góc
        gocBantay --;
        gocBantay = constrain(gocBantay, bantayMin, bantayMax);
        bantayServo.write(gocBantay);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 6 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocBantay;
            soChuyenDongTay ++;
          }
        } 
      }
    }
    
    // Tăng góc khủy tay
    if(dataIn == 7){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 2; // Chuyển động khủy tay
          gocBatDau[soChuyenDongTay] = gocKhuytay;
        }
      }

      // Tăng liên tục cho tới khi có tín hiệu dừng
      while(dataIn == 7){
        // Tăng góc
        gocKhuytay ++;
        gocKhuytay = constrain(gocKhuytay, khuytayMin, khuytayMax);
        khuytayServo.write(gocKhuytay);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 7 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocKhuytay;
            soChuyenDongTay ++;
          }
        }
      }
    }
    

    // Giảm góc khủy tay
    if(dataIn == 8){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 2; // Chuyển động khủy tay
          gocBatDau[soChuyenDongTay] = gocKhuytay;
        }
      }

      while(dataIn == 8){
        // Giảm góc
        gocKhuytay --;
        gocKhuytay = constrain(gocKhuytay, khuytayMin, khuytayMax);
        khuytayServo.write(gocKhuytay);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 8 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocKhuytay;
            soChuyenDongTay ++;
          }
        }
      } 
    }

    // Tăng góc vai
    if(dataIn == 9){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 3; // Chuyển động vai
          gocBatDau[soChuyenDongTay] = gocVai;
        }
      }

      while(dataIn == 9){
        // Tăng góc
        gocVai ++;
        gocVai = constrain(gocVai, vaiMin, vaiMax);
        vaiServo.write(gocVai);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 9 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocVai;
            soChuyenDongTay ++;
          }
        }
      }
    }
    
    // Giảm góc vai
    if(dataIn == 10){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 3; // Chuyển động vai
          gocBatDau[soChuyenDongTay] = gocVai;
        }
      }

      while(dataIn == 10){
        // Giảm góc
        gocVai --;
        gocVai = constrain(gocVai, vaiMin, vaiMax);
        vaiServo.write(gocVai);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 10 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocVai;
            soChuyenDongTay ++;
          }
        }
      }
    }
    

    // Tăng góc hông
    if(dataIn == 11){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 4; // Chuyển động hông
          gocBatDau[soChuyenDongTay] = gocHong;
        }
      }

      while(dataIn == 11){
        // Tăng góc
        gocHong ++;
        gocHong = constrain(gocHong, hongMin, hongMax);
        hongServo.write(gocHong);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 11 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocHong;
            soChuyenDongTay ++;
          }
        }
      }
    }
    

    // Giảm góc hông
    if(dataIn == 12){
      // Nếu đang record
      if(isRecordingArm){
        // Nếu chưa đạt tới số chuyển động tối đa
        if(soChuyenDongTay < MAX_RECORDING_LENGTH){
          dsChuyenDongTay[soChuyenDongTay] = 4; // Chuyển động hông
          gocBatDau[soChuyenDongTay] = gocHong;
        }
      }

      while(dataIn == 12){
        // Giảm góc
        gocHong --;
        gocHong = constrain(gocHong, hongMin, hongMax);
        hongServo.write(gocHong);
        delay(armDelay);
        
        // Kiểm tra tín hiệu gửi đến
        // Khi người dùng nhả phím điều khiển, app sẽ gửi tới một mã dừng (99)
        // Hoặc người dùng gửi mã khác
        if(Bluetooth.available() > 0){
          dataIn = Bluetooth.read();

          // Kết thúc chuyển động
          if(dataIn != 12 && isRecordingArm && soChuyenDongTay < MAX_RECORDING_LENGTH){
            gocKetThuc[soChuyenDongTay] = gocHong;
            soChuyenDongTay ++;
          }
        }
      }
    }
    

    /* ------------------- Chức năng phụ khác -------------------- */
    // Né vật cản
    if(dataIn == 13){
      // Thu cánh tay lại, tránh trường hợp cánh tay cản dò line
      resetArm();

      // Bắt đầu di chuyển và né vật cản
      tranhVatCan();

      // Dừng xe khi kết thúc
      dungXe();
    }

    // 14 là dừng né vật cản hoặc dùng bất kỳ nút nào khác để dừng

    /* ------------------- Chức năng record -------------------- */

    /* ------------------- Record cánh tay -------------------- */
    // Bắt đầu ghi
    if (dataIn == 15){
      // Nếu không đang ghi
      if(!isRecordingArm){
        // Reset tay
        resetArm();
        
        // Bật ghi
        isRecordingArm = 1;

        // Xóa thao tác đã lưu trước đó
        soChuyenDongTay = 0;
      }
    }

    // Dừng ghi
    if(dataIn == 16){
      isRecordingArm = 0;
    }

    // Phát những chuyển động đã ghi
    if(dataIn == 17){
      // Reset vị trí cánh tay
      resetArm();

      int stopCon = 0;
      
      // Với mỗi chuyển động
      for (i = 0; i < soChuyenDongTay && !stopCon; i++){
        // Chuyển động bàn tay
        if(dsChuyenDongTay[i] == 1){

          // Tăng góc
          if(gocBatDau[i] < gocKetThuc[i]){
            for (j = gocBatDau[i]; j <= gocKetThuc[i]; j++){
              gocBantay = j;
              bantayServo.write(gocBantay);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }

          // Giảm góc
          else{
            for (j = gocBatDau[i]; j >= gocKetThuc[i]; j--){
              gocBantay = j;
              bantayServo.write(gocBantay);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }
        }

        // Chuyển động khủy tay
        if(dsChuyenDongTay[i] == 2){

          // Tăng góc
          if(gocBatDau[i] < gocKetThuc[i]){
            for (j = gocBatDau[i]; j <= gocKetThuc[i]; j++){
              gocKhuytay = j;
              khuytayServo.write(gocKhuytay);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }

          // Giảm góc
          else{
            for (j = gocBatDau[i]; j >= gocKetThuc[i]; j--){
              gocKhuytay = j;
              khuytayServo.write(gocKhuytay);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }
        }

        // Chuyển động vai
        if(dsChuyenDongTay[i] == 3){

          // Tăng góc
          if(gocBatDau[i] < gocKetThuc[i]){
            for (j = gocBatDau[i]; j <= gocKetThuc[i]; j++){
              gocVai = j;
              vaiServo.write(gocVai);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }

          // Giảm góc
          else{
            for (j = gocBatDau[i]; j >= gocKetThuc[i]; j--){
              gocVai = j;
              vaiServo.write(gocVai);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }
        }

        // Chuyển động hông
        if(dsChuyenDongTay[i] == 4){

          // Tăng góc
          if(gocBatDau[i] < gocKetThuc[i]){
            for (j = gocBatDau[i]; j <= gocKetThuc[i]; j++){
              gocHong = j;
              hongServo.write(gocHong);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }

          // Giảm góc
          else{
            for (j = gocBatDau[i]; j >= gocKetThuc[i]; j--){
              gocHong = j;
              hongServo.write(gocHong);
              delay(armDelay);

              // Kiểm tra tín hiệu điều khiển
              if(Bluetooth.available()){
                dataIn = Bluetooth.read();

                if(dataIn != 17){
                  stopCon = 1;
                  break;
                }
              }
            }
          }
        }

        delay(500);
        
      }
    }

    /* -------------------    Record xe    -------------------- */
    // Bắt đầu ghi
    if(dataIn == 18){
      
    }

    // Dừng ghi
    if(dataIn == 19){
      
    }

    // Phát
    if(dataIn == 20){
      
    }
  }
}
