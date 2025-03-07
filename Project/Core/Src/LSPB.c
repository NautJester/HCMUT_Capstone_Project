#include "LSPB.h"

// Hàm khởi tạo LSPB với các thông số đầu vào
void LSPB_Init(LSPB_Params* lspb, float q0, float qf, float v_max, float a_max, float t_total)
{
    lspb->q0 = q0;
    lspb->qf = qf;
    lspb->t_total = t_total;
    lspb->v_max = v_max;
    lspb->a_max = a_max;


// Accelaration time and constant time
    lspb->t_acc = lspb->v_max / lspb->a_max;
    lspb->t_const = lspb->t_total - 2 * lspb->t_acc;
}

// Calculate angle based on LSPB
float LSPB_CalculatePosition(LSPB_Params* lspb, float t) {
    float q;

    if (lspb->qf - lspb->q0 > 0)
    {
        // Initial blend
        if (t < lspb->t_acc)
        {
            q = lspb->q0 + 0.5f * lspb->a_max * t * t;
        }
        // Constant velocity
        else if (t < (lspb->t_acc + lspb->t_const))
        {
            q = lspb->q0 + 0.5f * lspb->a_max * lspb->t_acc * lspb->t_acc +
                lspb->v_max * (t - lspb->t_acc);
        }
        // Final blend
        else if (t < lspb->t_total)
        {
            float t_dec = t - lspb->t_acc - lspb->t_const;
            q = lspb->q0 + 0.5f * lspb->a_max * lspb->t_acc * lspb->t_acc +
                lspb->v_max * lspb->t_const +
                lspb->v_max * t_dec - 0.5f * lspb->a_max * t_dec * t_dec;
        }
        // End journey
        else
        {
            q = lspb->qf;
        }
    }

    else if(lspb->qf - lspb->q0 < 0)
    {
        // Initial blend
        if (t < lspb->t_acc)
        {
            q = lspb->q0 - 0.5f * lspb->a_max * t * t;
        }
        // Constant velocity
        else if (t < (lspb->t_acc + lspb->t_const))
        {
            q = lspb->q0 - 0.5f * lspb->a_max * lspb->t_acc * lspb->t_acc -
                lspb->v_max * (t - lspb->t_acc);
        }
        // Final blend
        else if (t < lspb->t_total)
        {
            float t_dec = t - lspb->t_acc - lspb->t_const;
            q = lspb->q0 - 0.5f * lspb->a_max * lspb->t_acc * lspb->t_acc -
                lspb->v_max * lspb->t_const -
                lspb->v_max * t_dec + 0.5f * lspb->a_max * t_dec * t_dec;
        }
        // End journey
        else
        {
            q = lspb->qf;
        }
    }
    else
    {
    	q = lspb->qf;
    }

    return q;
}

