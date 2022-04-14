#include "m_pd.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>

#define ALT_FPGASLVS_OFST         0xc0000000
#define GENERIC_APB_IO_0_BASE           0x00
#define GENERIC_APB_IO_0_SPAN           32

#define IN_SMP_BUF_BASE           0x4000
#define IN_SMP_BUF_SPAN           0x1000

#define OUT_SMP_BUF_BASE          0x5000
#define OUT_SMP_BUF_SPAN          0x1000


EXTERN int* get_sys_sleepgrain(void);

static t_class *fpga_timing_measurements_tilde_class;

typedef struct _fpga_timing_measurements_tilde {
  t_object  x_obj;
  t_outlet* f_out2;
  t_outlet* f_out3;
  t_outlet* f_out4;
  t_sample  f_dummy;

  int fd;
  void * in_smp_buf_map;
  int *  in_smp_buf_ptr;
  void * out_smp_buf_map;
  int *  out_smp_buf_ptr;

  long tv_sec_past;
  long tv_nsec_past;
  float f_time_diff_us;
  long fpga_clk_steps;
  struct timespec tp;

} t_fpga_timing_measurements_tilde;

t_int *fpga_timing_measurements_tilde_perform(t_int *w) {
  t_fpga_timing_measurements_tilde *x = (t_fpga_timing_measurements_tilde *)(w[1]);
  t_sample  *audio_in  =    (t_sample *)(w[2]);
  t_sample  *audio_out =    (t_sample *)(w[3]);
  int          n =           (int)(w[4]);
  
  x->fpga_clk_steps = x->out_smp_buf_ptr[1]; // clock steps from hardware counter on FPGA  
  outlet_float(x->f_out4, (t_float) x->fpga_clk_steps / 50.0f); // converts from steps to us

  if (x->out_smp_buf_ptr[0] != 0xabcd) {
    // fpga not finished
    pd_error(x, "FPGA buffer dropout detected");
  }


  /*for(int i = 0; i<n; i++) {
    x->in_smp_buf_ptr[(int)512+i] =  audio_in[i]*(t_int)(1<<31);
    audio_out[i] = x->out_smp_buf_ptr[(int)512+i]/(t_sample)(1<<31);; 
  }*/
  // copy data to fpga
  memcpy((void*)&x->in_smp_buf_ptr[512], (const void*)audio_in, 4*n);
  // get old (processed) data back from FPGA
  memcpy((void*)audio_in, (const void*)&x->out_smp_buf_ptr[512], 4*n);

  x->in_smp_buf_ptr[1] = n;
  x->in_smp_buf_ptr[0] = 0xabcd; // buffer ready signal
  x->out_smp_buf_ptr[0] = 0x0; // FPGA not ready

  

  return (w+6);
}

void fpga_timing_measurements_tilde_dsp(t_fpga_timing_measurements_tilde *x, t_signal **sp) {
  dsp_add(fpga_timing_measurements_tilde_perform, 5, x,
          sp[0]->s_vec, sp[1]->s_vec, (t_int)sp[0]->s_n);
}

void fpga_timing_measurements_tilde_free(t_fpga_timing_measurements_tilde *x){
  outlet_free(x->f_out2);
  outlet_free(x->f_out3);
  outlet_free(x->f_out4);
  munmap(x->in_smp_buf_map, IN_SMP_BUF_SPAN);
  munmap(x->out_smp_buf_map, OUT_SMP_BUF_SPAN);
}

void *fpga_timing_measurements_tilde_new(void) {
  t_fpga_timing_measurements_tilde *x = (t_fpga_timing_measurements_tilde *)pd_new(fpga_timing_measurements_tilde_class);

  outlet_new(&x->x_obj, &s_signal);
  x->f_out2 = outlet_new(&x->x_obj, &s_float);
  x->f_out3 = outlet_new(&x->x_obj, &s_float);
  x->f_out4 = outlet_new(&x->x_obj, &s_float);
  x->fd = open("/dev/mem", (O_RDWR | O_SYNC));

  x->in_smp_buf_map = mmap( NULL, IN_SMP_BUF_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + IN_SMP_BUF_BASE) );
  x->in_smp_buf_ptr = (int *)(x->in_smp_buf_map);


  x->out_smp_buf_map = mmap( NULL, OUT_SMP_BUF_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + OUT_SMP_BUF_BASE) );
  x->out_smp_buf_ptr = (int *)(x->out_smp_buf_map);

  // variables for time measurement
  x->tv_sec_past = 0;
  x->tv_nsec_past = 0;
  x->fpga_clk_steps = 0;
  post("fpga_timing_measurements~ • Constructor was called");

  return (void *)x;
}


static void set_sleepgrain(t_fpga_timing_measurements_tilde *x, t_float f)
{
  int value=(int)f;
  int*current=get_sys_sleepgrain();

  if(value<=0) {
    pd_error(x, "[fpga_timing_measurements]: sleepgrain cannot be <= 0");
    return;
  }

  *current=value;

  //  outlet_float(x->x_obj.ob_outlet, f);
}

static void get_sleepgrain(t_fpga_timing_measurements_tilde *x)
{
  int*current=get_sys_sleepgrain();
  t_float f=*current;
  outlet_float(x->f_out3, f);
}



void fpga_timing_measurements_tilde_setup(void) {
  fpga_timing_measurements_tilde_class = class_new(gensym("fpga_timing_measurements~"),
        (t_newmethod)fpga_timing_measurements_tilde_new,
        (t_method)fpga_timing_measurements_tilde_free,
         sizeof(t_fpga_timing_measurements_tilde),
        CLASS_DEFAULT, 0);

  class_addmethod(fpga_timing_measurements_tilde_class, (t_method)fpga_timing_measurements_tilde_dsp, gensym("dsp"), A_CANT);
  CLASS_MAINSIGNALIN(fpga_timing_measurements_tilde_class, t_fpga_timing_measurements_tilde, f_dummy);


  class_addmethod(fpga_timing_measurements_tilde_class,(t_method)get_sleepgrain, gensym("getsleepgrain"), 0);


  class_addmethod(fpga_timing_measurements_tilde_class,(t_method)set_sleepgrain, gensym("setsleepgrain"),A_DEFFLOAT, 0);

  /* Print message to PD window */
  post("fpga_timing_measurements~ • External was loaded");
}