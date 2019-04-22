#ifndef mtr
#define mtr

class Motor {
    private:
    int R_EN;
    int R_PWM;
    int L_PWM;
    int L_EN;
   


    public:
    Motor( int r_en, int r_pwm, int l_en, int l_pwm);
    void init();
    void move(char a, char b);
    void stop(void);

};
#endif