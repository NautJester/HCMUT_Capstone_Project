#ifndef INC_LSPB_H_
#define INC_LSPB_H_


#define DOF 5 // Số bậc tự do

typedef struct {
    float t_acc;    // Thời gian tăng tốc
    float t_const;  // Thời gian chuyển động với vận tốc không đổi
    float t_total;  // Tổng thời gian chuyển động
    float v_max;    // Vận tốc cực đại
    float a_max;    // Gia tốc cực đại
    float q0;       // Vị trí ban đầu
    float qf;       // Vị trí cuối
} LSPB_Params;

void LSPB_Init(LSPB_Params* lspb, float q0, float qf, float v_max, float a_max, float t_total);
float LSPB_CalculatePosition(LSPB_Params* lspb, float t);




#endif /* INC_LSPB_H_ */
