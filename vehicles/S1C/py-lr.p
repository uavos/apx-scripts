#define NODE_LEFT               TRUE
//#define NODE_RIGHT              TRUE


const idx_gpio1 = f_VM1;
const idx_gpio2 = f_VM2;
const idx_gpio3 = f_VM3;
const idx_gpio4 = f_VM4;

const idx_adc1 = f_VM11;
const idx_adc2 = f_VM12;
const idx_adc3 = f_VM13;

new Float: adc[3];
new Float: gpio_fb[2];

new ch1_timer = 0;
new ch2_timer = 0;

main()
{


}

@OnTask()
{

  adc[0] = get_var(idx_adc1);
  adc[1] = get_var(idx_adc2);
  adc[2] = get_var(idx_adc3);

  new gpio = get_var(idx_gpio1);
  if (gpio != gpio_fb[0]) {
      gpio_fb[0] = gpio;
      printf("PY-1:%d", gpio);
  }

  gpio = get_var(idx_gpio2);
  if (gpio != gpio_fb[1]) {
      gpio_fb[1] = gpio;
      printf("PY-2:%d", gpio);
  }

  if(time() - ch1_timer > 1000) {
      set_var(idx_gpio3, 0.0, false);
  }

  if(time() - ch2_timer > 1000) {
      set_var(idx_gpio4, 0.0, false);
  }

  return 100;
}


//vmexec('@vm_drop_all')    //Console
forward @vm_drop_all()
@vm_drop_all()
{
    printf("DROP-ALL:\n");
    set_var(idx_gpio3, 1.0, false);
    set_var(idx_gpio4, 1.0, false);
    ch1_timer = time();
    ch2_timer = time();
}

#if defined NODE_LEFT
//vmexec('@vm_pyl')     //Console
forward @vm_pyl()
@vm_pyl()
{
    printf("L: %.1f : %.1f : %.1f", adc[0], adc[1], adc[2]);

    new st1 = get_var(idx_gpio1);
    new st2 = get_var(idx_gpio2);
    printf("L: %d : %d", st1, st2);
}

//vmexec('@vm_py1')    //Console
forward @vm_py1()
@vm_py1()
{
    printf("DROP-1:\n");
    set_var(idx_gpio3, 1.0, false);
    ch1_timer = time();
}

//vmexec('@vm_py2')    //Console
forward @vm_py2()
@vm_py2()
{
    printf("DROP-2:\n");
    set_var(idx_gpio4, 1.0, false);
    ch2_timer = time();
}
#endif


#if defined NODE_RIGHT
//vmexec('@vm_pyr')     //Console
forward @vm_pyr()
@vm_pyr()
{
    printf("R: %.1f : %.1f : %.1f", adc[0], adc[1], adc[2]);

    new st3 = get_var(idx_gpio1);
    new st4 = get_var(idx_gpio2);
    printf("R: %d : %d", st3, st4);
}

//vmexec('@vm_py3')    //Console
forward @vm_py3()
@vm_py3()
{
    printf("DROP-3:\n");
    set_var(idx_gpio3, 1.0, false);
    ch1_timer = time();
}

//vmexec('@vm_py4')    //Console
forward @vm_py4()
@vm_py4()
{
    printf("DROP-4:\n");
    set_var(idx_gpio4, 1.0, false);
    ch2_timer = time();

}
#endif
