#define DCDC1		0x21
#define DCDC2		0x22
#define DCDC3		0x23
#define DCDC4		0x24
#define DCDC5		0x25

#define ALDO1		0x28
#define ALDO2		0x29
#define ALDO3		0x2A

#define ELDO1		0x19
#define ELDO2		0x1A
#define ELDO3		0x1B

#define DLDO1		0x15
#define DLDO2		0x16
#define DLDO3		0x17
#define DLDO4		0x18

#define DC5LDO	0x1C

#define GPIO0		0x90
#define LDOIO0	0x91
#define GPIO1		0x92

#define PWREN1	0x8C 
#define PWREN2	0x8D 

#define LDO1_EN	0x10
#define LDO2_EN	0x12
#define ALDO3_EN	0x13

#define POWER_KEY	0x4A
#define SHORT_KEY	0x02
#define LONG_KEY	0x01

#define BAT_SOC	0xB9 
#define POWER_OFF_REG  	0x32

#define CHARGE_REG1	0x33
#define CHARGE_REG2	0x34
#define CHARGE_REG3	0x35

#define AXP22_VOFF_SET		0x31
#define AXP22_HOTOVER_CTL       0x8F
#define AXP22_IPS_SET           0x30
#define AXP22_MODE_CHGSTATUS    0x01
#define AXP22_STATUS            0x00

#define AXP22_BATCAP0                    (0xe0)
#define AXP22_BATCAP1                    (0xe1)

#define OCVREG0				0x00		 //2.99V
#define OCVREG1				0x00		 //3.13V
#define OCVREG2				0x00		 //3.27V
#define OCVREG3				0x00		 //3.34V
#define OCVREG4				0x00		 //3.41V
#define OCVREG5				0x00		 //3.48V
#define OCVREG6				0x00		 //3.52V
#define OCVREG7				0x00		 //3.55V
#define OCVREG8				0x04		 //3.57V
#define OCVREG9				0x05		 //3.59V
#define OCVREGA				0x06		 //3.61V
#define OCVREGB				0x07		 //3.63V
#define OCVREGC				0x0a		 //3.64V
#define OCVREGD				0x0d		 //3.66V
#define OCVREGE				0x1a		 //3.7V 
#define OCVREGF				0x24		 //3.73V
#define OCVREG10		 	0x29                //3.77V
#define OCVREG11		 	0x2e                //3.78V
#define OCVREG12		 	0x32                //3.8V 
#define OCVREG13		 	0x35                //3.84V
#define OCVREG14		 	0x39                //3.85V
#define OCVREG15		 	0x3d                //3.87V
#define OCVREG16		 	0x43                //3.91V
#define OCVREG17		 	0x49                //3.94V
#define OCVREG18		 	0x4f                //3.98V
#define OCVREG19		 	0x54                //4.01V
#define OCVREG1A		 	0x58                //4.05V
#define OCVREG1B		 	0x5c                //4.08V
#define OCVREG1C		 	0x5e                //4.1V 
#define OCVREG1D		 	0x60                //4.12V
#define OCVREG1E		 	0x62                //4.14V
#define OCVREG1F		 	0x64                //4.15V

/*电池容量，mAh：根据实际电池容量来定义，对库仑计方法来说
这个参数很重要，必须配置*/
#define BATCAP				7200

extern void axp229_set_voltage(uint8_t ldo,unsigned int mv,uint8_t onoff);
extern void axp229_power_off(void);
extern void axp229_set_charge_current(unsigned int ma);
void axp229_set_arm_voltage(unsigned int target_freq);
