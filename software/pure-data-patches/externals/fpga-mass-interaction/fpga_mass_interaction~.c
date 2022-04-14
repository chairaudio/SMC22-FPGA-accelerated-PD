#include "m_pd.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>

#define ALT_FPGASLVS_OFST         0xc0000000
#define GENERIC_APB_IO_0_BASE           0x00
#define GENERIC_APB_IO_0_SPAN           32

#define MI_STATE_VARIABLE_BUFFER_BASE      	0x2000
#define MI_STATE_VARIABLE_BUFFER_SPAN      	0x1000

#define IN_SMP_BUF_BASE						0x4000
#define IN_SMP_BUF_SPAN      				0x1000

#define OUT_SMP_BUF_BASE					0x5000
#define OUT_SMP_BUF_SPAN      				0x1000

#define N_MASSES 25

EXTERN int* get_sys_sleepgrain(void);

void connect_masses_to_string(uint32_t *flat_mat, int n_masses, uint32_t q31_tension);
uint32_t tension_ftoq31(t_sample f_tension);

static t_class *fpga_mass_interaction_tilde_class;


typedef struct _fpga_mass_interaction_tilde {
  t_object  x_obj;
  t_inlet *x_in2;
  t_inlet *x_in3;
  t_outlet*x_out;
  t_sample f;
  t_sample f_tension;
  int fd;
  int * apb_io_ptr;
  void * mi_mem_map;
  uint32_t * mi_mem_ptr;
  void * in_smp_buf_map;
  int *  in_smp_buf_ptr;
  int * in_smp_buf_new_samples_written;
  int * in_smp_buf_size;
  void * out_smp_buf_map;
  int *  out_smp_buf_ptr;
} t_fpga_mass_interaction_tilde;

t_int *fpga_mass_interaction_tilde_perform(t_int *w) {

  
  t_fpga_mass_interaction_tilde *x = (t_fpga_mass_interaction_tilde *)(w[1]);
  t_sample  *audio_in =    (t_sample *)(w[2]);
  t_sample  *audio_out =    (t_sample *)(w[3]);
  int          n =           (int)(w[4]);
  const float Q27_MAX_F =  0x0.FFFFFFp0F;
  const float Q27_MIN_F = -1.0F;

  if (x->out_smp_buf_ptr[0] != 0xabcd) {
    // fpga not finished
    pd_error(x, "FPGA buffer dropout detected");
  }


  connect_masses_to_string(x->mi_mem_ptr, N_MASSES, tension_ftoq31(x->f_tension));

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

void fpga_mass_interaction_tilde_dsp(t_fpga_mass_interaction_tilde *x, t_signal **sp) {
  dsp_add(fpga_mass_interaction_tilde_perform, 4, x,
          sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_n);
}

void fpga_mass_interaction_tilde_free(t_fpga_mass_interaction_tilde *x){
  inlet_free(x->x_in2);
  inlet_free(x->x_in3);
  outlet_free(x->x_out);
  munmap(x->in_smp_buf_map, IN_SMP_BUF_SPAN);
  munmap(x->out_smp_buf_map, OUT_SMP_BUF_SPAN);
  munmap(x->mi_mem_map, MI_STATE_VARIABLE_BUFFER_SPAN);
}

void *fpga_mass_interaction_tilde_new(t_floatarg f) {
  t_fpga_mass_interaction_tilde *x = (t_fpga_mass_interaction_tilde *)pd_new(fpga_mass_interaction_tilde_class);
  x->f_tension = f;
  x->x_in2 = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  x->x_in3 = floatinlet_new (&x->x_obj, &x->f_tension);
  x->x_out = outlet_new(&x->x_obj, &s_signal);

  x->fd = open("/dev/mem", (O_RDWR | O_SYNC));

  x->mi_mem_map = mmap( NULL, MI_STATE_VARIABLE_BUFFER_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + MI_STATE_VARIABLE_BUFFER_BASE) );
  x->mi_mem_ptr = (uint32_t *) (x->mi_mem_map);

  x->in_smp_buf_map = mmap( NULL, IN_SMP_BUF_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + IN_SMP_BUF_BASE) );
  x->in_smp_buf_ptr = (int *)(x->in_smp_buf_map);


  x->out_smp_buf_map = mmap( NULL, OUT_SMP_BUF_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, x->fd, (ALT_FPGASLVS_OFST + OUT_SMP_BUF_BASE) );
  x->out_smp_buf_ptr = (int *)(x->out_smp_buf_map);

  connect_masses_to_string(x->mi_mem_ptr, N_MASSES, tension_ftoq31(x->f_tension));
  post("fpga_mass_interaction~ • Construcor was called");

  return (void *)x;
}

static void set_sleepgrain(t_fpga_mass_interaction_tilde *x, t_float f)
{
  int value=(int)f;
  int*current=get_sys_sleepgrain();

  if(value<=0) {
    pd_error(x, "[fpga_timing_measurements]: sleepgrain cannot be <= 0");
    return;
  }
  *current=value;
}


void fpga_mass_interaction_tilde_setup(void) {
  fpga_mass_interaction_tilde_class = class_new(gensym("fpga_mass_interaction~"),
        (t_newmethod)fpga_mass_interaction_tilde_new,
        0, sizeof(t_fpga_mass_interaction_tilde),
        CLASS_DEFAULT,
        A_DEFFLOAT, 0);

  class_addmethod(fpga_mass_interaction_tilde_class,
        (t_method)fpga_mass_interaction_tilde_dsp, gensym("dsp"), A_CANT, 0);
  CLASS_MAINSIGNALIN(fpga_mass_interaction_tilde_class, t_fpga_mass_interaction_tilde, f);

  class_addmethod(fpga_mass_interaction_tilde_class,(t_method)set_sleepgrain, gensym("setsleepgrain"), A_DEFFLOAT, 0);

  /* Print message to PD window */
  post("fpga_mass_interaction~ • External was loaded");
}


void connect_masses_to_string(uint32_t *flat_mat, int n_masses, uint32_t tension) {

	if(n_masses>43) {
		n_masses = 43;
		printf("Maximum number of Masses is 43! Setting to 43!\n");
	}

	int n_connections = (n_masses*(n_masses-1))/2;
    //int tension =  0x000FFFFF; // max: 0x003FFFFF;
    int row_width = n_masses;
    int row_idx = 0;
    //printf("setting up flat connection matrix with %d connections...\n", n_connections);
    for (int i = 0; i < n_connections; i = i + 1) {

        if ( (row_idx%row_width) == 0) 
        {
            flat_mat[i] = tension;
            row_width -=1;
            row_idx = 0;
            //printf("1, ");
        }
        else {
            flat_mat[i]= 0;
            //printf("0, ");
        }
        row_idx += 1;
    }
    //printf("\n");
}

uint32_t tension_ftoq31(t_sample f_tension) {
  uint32_t q31_tension = (f_tension<=0)? 0x0000FFFF :(f_tension>1)?0x003FFFFF:(uint32_t)(f_tension*0x003FFFFF);
  return q31_tension;
}


