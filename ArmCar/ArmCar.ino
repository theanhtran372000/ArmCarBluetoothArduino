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

// Cảm biến dò line
const int rline = 8;  // Dò line bên phải
const int mline = 9;  // Dò line giữa
const int lline = 10; // Dò line bên trái

// Chân cảm biến siêu âm đo khoảng cách
const int echo = 12;  // Chân đo thời gian
const int trig = 13;  // Chân phát sóng

/* ------------------- Khai báo biến + hằng ------------------- */
// Hằng
const int armDelay = 10 ;   // Thời gian delay giữa 2 lần điều khiển
const int avoidDelay = 20;  // Thời gian delay giữa 2 lần đo khoảng cách
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

/* --------------- Hàm dò line, tránh vật cản ----------------- */
// Hàm dò line
void doLine(){
  int lech = 0;
  
  do{
    // Nếu đang nằm trên line
    if(digitalRead(mline)){
      tienXe(150); // Đi với tốc độ vừa phải để dò line
    }
    
    // Đang lệch sang bên trái do dò được line bên phải
    else if(digitalRead(rline)){
      // Rẽ phải tới khi không còn dò được line bên phải nữa
      lech = 1;
      rePhai(150);
      while(digitalRead(rline));
    }

    // Đang lệch sang bên phải do dò được line ở bên trái
    else if(digitalRead(lline)){
      // Rẽ trái tới khi không còn dò được line bên trái nữa
      lech = -1;
      reTrai(150);
      while(digitalRead(lline));
    }

    // Trường hợp không dò được line ở bất kỳ cảm biến nào
    // Nếu trước đó đang lệch trái (dò đc line bên phải)
    else if(lech == 1){
      rePhai(100); // Rẽ chậm để dò lại line
    }

    // Nếu trước đó đang lệch phải
    else if(lech == -1){
      reTrai(100);
    }

    // Cuối vòng lặp, kiểm tra tín hiệu đến
    if (Bluetooth.available() > 0){
      dataIn = Bluetooth.read();
    }
  } while (dataIn == 13); // Nếu nhận được lệnh khác thì dừng dò line
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
  
  return min(khoangcach, maxDistance); // giới hạn kết quả trả về (tối đa 300cm)
}

// Hàm tránh vật cảm
void tranhVatCan(){
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

      // Quay lại hướng cũ (khoảng 90 độ)
      rePhai(200);
      delay(500);
      dungXe();
      delay(500);

      // Kiểm tra khoảng cách
      // Một trong 2 bên trống
      if(khoangCachTrai > minDistance || khoangCachPhai > minDistance){
        // Bên trái đường rộng rãi hơn
        if(khoangCachTrai > khoangCachPhai){
          reTrai(200);
          delay(500);
          dungXe();
          delay(500);
        }

        // Ngược lại
        else{
          rePhai(200);
          delay(500);
          dungXe();
          delay(500);
        }
      }
      
      // Nếu cả 2 bên đều không có đường thì quay xe
      else{
        rePhai(200);
        delay(1000);
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
  }while(dataIn == 14);
}

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

  // Chân dò line
  pinMode(rline, INPUT);
  pinMode(mline, INPUT);
  pinMode(lline, INPUT);

  // Chân cảm biến sóng âm
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Thiết lập servo
  // Phần hông quay
  hongServo.attach(hongPin, 450, 2500); // mở rộng góc quay (0 -> 180) tính ra được thời gian ms: 450 -> 2500ms
  hongMin = 0;
  hongMax = 180;
  gocHong = 90;
  hongServo.write(gocHong);

  // Phần vai
  vaiServo.attach(vaiPin);
  vaiMin = 0;
  vaiMax = 60;
  gocVai = 0;
  vaiServo.write(gocVai);
  
  // Phần khủy tay
  khuytayServo.attach(khuytayPin);
  khuytayMin = 0;
  khuytayMax = 60;
  gocKhuytay = 10;
  khuytayServo.write(gocKhuytay);

  // Phần bàn tay
  bantayServo.attach(bantayPin);
  bantayMin = 20;
  bantayMax = 40;
  gocBantay = 40;
  bantayServo.write(gocBantay);
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
      }
    }

    // Giảm góc bàn tay
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
      }
    }

    // Tăng góc khủy tay
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
      }
    }

    // Giảm góc khủy tay
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
      }
    }

    // Tăng góc vai
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
      }
    }

    // Giảm góc vai
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
      }
    }

    // Tăng góc hông
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
      }
    }

    // Giảm góc hông
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
      }
    }

    /* ------------------- Chức năng phụ khác -------------------- */
    // Dò line
    if(dataIn == 13){
      // Bắt đầu do line
      doLine();

      // Dừng xe khi dò line kết thúc
      dungXe();
    }

    // Né vật cản
    if(dataIn == 14){
      // Bắt đầu di chuyển và né vật cản
      tranhVatCan();

      // Dừng xe khi kết thúc
      dungXe();
    }
  }
}
