#include <FlexCAN_T4.h>

struct DriverCommand {
  uint8_t kp;
  uint8_t kv;
  uint32_t q_d;
  int16_t q_dot_d;
  int16_t tau_ff;
};

union StatePacket{
	struct{
		int32_t q;
		int16_t q_dot;
		int16_t tau;
	}data;
	uint8_t buffer[8];
};

union  StatePacket   can1_states[1];
struct DriverCommand can1_cmds[1];
CAN_message_t can1_cmd_msgs[1];

void packCmdMessage(CAN_message_t *msg, struct DriverCommand *cmd)
{
  uint32_t q_d = 2097151 + cmd->q_d;
  int16_t q_dot_d = 511 + cmd->q_dot_d;
  msg->buf[0]=cmd->kp;
  msg->buf[1]=cmd->kv;
  msg->buf[7] = 0xff&cmd->tau_ff;
  msg->buf[6] = cmd->tau_ff >> 8;
  msg->buf[2] = (q_dot_d>>2)&0xff;
  msg->buf[3] = (q_dot_d& 0b11)<<6;
  msg->buf[3] |= ((q_d >> (16)) & 0b00111111);
  msg->buf[4]  = (q_d >> (8))&0xff;
  msg->buf[5]  = (q_d)&0xff;
}

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;
bool transmit_flag = false;
void daq_transaction_callback()
{
  transmit_flag = true;
}

IntervalTimer daqLoggerTimer;

void setup(void) {
  Serial.begin(115200); delay(400);
  pinMode(6, OUTPUT); digitalWrite(6, LOW); /* optional tranceiver enable pin */
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();
  Can0.onReceive(canSniff);
  Can0.mailboxStatus();
  daqLoggerTimer.begin(daq_transaction_callback, 1000);
  
}
int print_counter = 0;
void canSniff(const CAN_message_t &msg) {
  print_counter++;
  if(print_counter>50)
  {
    print_counter =0;
    if(false)
    {
      Serial.print("MB "); Serial.print(msg.mb);
      // Serial.print("  OVERRUN: "); Serial.print(msg.flags.overrun);
      // Serial.print("  LEN: "); Serial.print(msg.len);
      // Serial.print(" EXT: "); Serial.print(msg.flags.extended);
      // Serial.print(" TS: "); Serial.print(msg.timestamp);
      Serial.print(" ID: "); Serial.print(msg.id, HEX);
      // Serial.print(" Buffer: ");
      for ( uint8_t i = 0; i < msg.len; i++ ) {
        // Serial.print(msg.buf[i], HEX); Serial.print(" ");
        can1_states[0].buffer[i] = msg.buf[i];
      }
      Serial.print(" q: "); Serial.print(can1_states[0].data.q);
      Serial.print(" q_dot: "); Serial.print(can1_states[0].data.q_dot);
      Serial.print(" tau: "); Serial.print(can1_states[0].data.tau);
      Serial.println();
    }
    else
    {
      for ( uint8_t i = 0; i < msg.len; i++ ) {
        can1_states[0].buffer[i] = msg.buf[i];
      }
      // Serial.print(can1_states[0].data.q);
      Serial.print(","); Serial.print(can1_states[0].data.q_dot);
      Serial.print(","); Serial.print(can1_states[0].data.tau);
      Serial.println();
    }
  }
}

uint64_t start_time = millis();
void loop() {
  Can0.events();
  if(transmit_flag)
  {
    float t = (float)(millis() - start_time)/1000.;
    // Serial.println(800*sin(t));
    transmit_flag = false;
    can1_cmds[0].q_d = t*100;1600+800*sin(t);
    can1_cmds[0].q_dot_d = 0;
    can1_cmds[0].tau_ff = 0;
    can1_cmds[0].kp = 5;
    can1_cmds[0].kv = 5;    
    packCmdMessage(&can1_cmd_msgs[0], &can1_cmds[0]);
    can1_cmd_msgs[0].id = 0x123;
    Can0.write(can1_cmd_msgs[0]);
  }
}
