#include "m_pd.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>

#define ALT_FPGASLVS_OFST        0xc0000000
#define GENERIC_APB_IO_0_BASE          0x00
#define GENERIC_APB_IO_0_SPAN            32


#define IN_SMP_BUF_BASE						   0x4000
#define IN_SMP_BUF_SPAN      				 0x1000

#define OUT_SMP_BUF_BASE					   0x5000
#define OUT_SMP_BUF_SPAN      		   0x1000


//EXTERN int* get_sys_sleepgrain(void);

static t_class *fpga_passthrough_tilde_class;


typedef struct _fpga_passthrough_tilde {
  t_object  x_obj;
  t_float f;
  t_outlet*x_out;
  t_inlet *x_in2;
  int fd;
  int * apb_io_ptr;
  void * in_smp_buf_map;
  volatile int *  in_smp_buf_ptr;
  void * out_smp_buf_map;
  volatile int *  out_smp_buf_ptr;
} t_fpga_passthrough_tilde;

t_int *fpga_passthrough_tilde_perform(t_int *w) {
  t_fpga_passthrough_tilde *x = (t_fpga_passthrough_tilde *)(w[1]);
  t_sample          *audio_in =                 (t_sample *)(w[2]);
  t_sample         *audio_out =                 (t_sample *)(w[3]);
  int                       n =                        (int)(w[4]);

  const float Q27_MAX_F =  0x0.FFFFFFp0F;
  const float Q27_MIN_F = -1.0F;

  if (x->out_smp_buf_ptr[0] != 0xabcd) {
    // fpga not finished
    pd_error(x, "FPGA buffer dropout detected");
  }

  for(int i = 0; i<n; i++){
    float to_pd_output = scalbnf(x->out_smp_buf_ptr[512+i], -31);  // the Cyclone V calculates with 27 bit signed integers in q26 format
    audio_out[i] = to_pd_output;

    float clamped = fmaxf(fminf(audio_in[i], Q27_MAX_F), Q27_MIN_F);
    int fpga_input = (int) scalbnf(clamped, 31); 
    x->in_smp_buf_ptr[512+i] =  fpga_input;
  }
  x->in_smp_buf_ptr[1] = n;
  x->in_smp_buf_ptr[0] = 0xabcd; // buffer ready signal
  x->out_smp_buf_ptr[0] = 0x0; // FPGA not ready

  return (w+5);
}

void fpga_passthrough_tilde_dsp(t_fpga_passthrough_tilde *x, t_signal **sp) {
  dsp_add(fpga_passthrough_tilde_perform, 4, x,
          sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_n);
}

void fpga_passthrough_tilde_free(t_fpga_passthrough_tilde *x){
  outlet_free(x->x_out);
  inlet_free(x->x_in2);
  munmap(x->in_smp_buf_map, IN_SMP_BUF_SPAN);
  munmap(x->out_smp_buf_map, OUT_SMP_BUF_SPAN);
}

void *fpga_passthrough_tilde_new(t_floatarg f) {
  post("fpga_passthrough~ • Construcor was called");

  t_fpga_passthrough_tilde *x = (t_fpga_passthrough_tilde *)pd_new(fpga_passthrough_tilde_class);
  x->x_out = outlet_new(&x->x_obj, &s_signal);
  x->x_in2 = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);

  x->f = 0;

  x->fd = open("/dev/mem", (O_RDWR | O_SYNC));

  x->in_smp_buf_map = mmap( NULL, IN_SMP_BUF_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + IN_SMP_BUF_BASE) );
  x->in_smp_buf_ptr = (int *)(x->in_smp_buf_map);

  x->out_smp_buf_map = mmap( NULL, OUT_SMP_BUF_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + OUT_SMP_BUF_BASE) );
  x->out_smp_buf_ptr = (int *)(x->out_smp_buf_map);

  post("fpga_passthrough~ • Object was built");

  return (void *)x;
}
/*
static void set_sleepgrain(t_fpga_passthrough_tilde *x, t_float f)
{
  int value=(int)f;
  int*current=get_sys_sleepgrain();

  if(value<=0) {
    pd_error(x, "[fpga_passthrough~]: sleepgrain cannot be <= 0");
    return;
  }
  *current=value;
}
*/

void fpga_passthrough_tilde_setup(void) {
  fpga_passthrough_tilde_class = class_new(gensym("fpga_passthrough~"),
        (t_newmethod)fpga_passthrough_tilde_new, 0, // contructor, destructor
        sizeof(t_fpga_passthrough_tilde), // size for memory allocation
        CLASS_DEFAULT, // graphical represantation
        A_DEFFLOAT, 0);

  class_addmethod(fpga_passthrough_tilde_class,
        (t_method)fpga_passthrough_tilde_dsp, gensym("dsp"), A_CANT, 0);
  CLASS_MAINSIGNALIN(fpga_passthrough_tilde_class, t_fpga_passthrough_tilde, f);

  //class_addmethod(fpga_passthrough_tilde_class,(t_method)set_sleepgrain, gensym("setsleepgrain"), A_DEFFLOAT, 0);

  /* Print message to PD window */
  post("fpga_passthrough~ • External was loaded");
}