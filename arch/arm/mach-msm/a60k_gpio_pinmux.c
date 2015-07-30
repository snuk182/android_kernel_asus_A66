#include <linux/kernel.h>
#include <mach/gpiomux.h>

#include "a60k_evb_gpio_pinmux.h"
#include "a60k_sr1_1_gpio_pinmux.h"

// +++ ASUS_BSP : Add for A66 Proj
#include "a66_sr1_gpio_pinmux.h"
// --- ASUS_BSP : Add for A66 Proj
#include "a66_er2_gpio_pinmux.h"

#define PM8921_GPIO_BASE        NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
#define PM8921_MPP_BASE         (PM8921_GPIO_BASE + PM8921_NR_GPIOS)
#define PM8921_MPP_PM_TO_SYS(pm_gpio)   (pm_gpio - 1 + PM8921_MPP_BASE)
#define PM8921_IRQ_BASE         (NR_MSM_IRQS + NR_GPIO_IRQS)
unsigned g_GPIO_HDMI_5V_ENABLE;//Mickey+++
unsigned g_GPIO_P01_I2C_LS_OE;
unsigned g_GPIO_MIC2_BIAS_EN;
unsigned g_GPIO_SPK_AMP_EN;
unsigned g_GPIO_HS_PATH_EN;
unsigned g_GPIO_HOOK_DET;
unsigned g_GPIO_JACK_IN_DET;
unsigned g_GPIO_LCD_TE;
unsigned g_GPIO_8M_WP;
unsigned g_GPIO_CAM_8M_RST_N;
unsigned g_GPIO_FLASH_ENT;

//+++ASUS_BSP Peter_lu : for set charging current to 1A..........................................
extern unsigned int ac_in;
//---ASUS_BSP Peter_lu : for set charging current to 1A..........................................

int __init a60k_gpio_init(void)
{   
   switch (g_A60K_hwID)
   {
          case A60K_EVB:
             printk("A60K gpio config table = EVB\n");
        g_GPIO_HDMI_5V_ENABLE = 4;//Mickey+++
		g_GPIO_P01_I2C_LS_OE = 3;
		g_GPIO_MIC2_BIAS_EN = 0;
        g_GPIO_SPK_AMP_EN = 46;
        g_GPIO_HS_PATH_EN = 18; 
        g_GPIO_HOOK_DET = 12; 
		g_GPIO_LCD_TE = 19;
		g_GPIO_JACK_IN_DET = (PM8921_GPIO_PM_TO_SYS(38));//Rice	 
	     msm_gpiomux_install(a60k_evb_msm8960_gpio_configs,
			ARRAY_SIZE(a60k_evb_msm8960_gpio_configs));	 	  
             break;
				 
          case A60K_SR1_1:
          case A60K_SR1_2_ES1:
          case A60K_SR1_2_ES2:
          case A60K_ER1:
          case A60K_ER2:
          case A60K_PR:
             printk("A60K gpio config table = SR1_1\n");
        g_GPIO_HDMI_5V_ENABLE = 4;//Mickey+++
		g_GPIO_P01_I2C_LS_OE = 33;
		g_GPIO_MIC2_BIAS_EN = 19;
        g_GPIO_SPK_AMP_EN = 46;
        g_GPIO_HS_PATH_EN = 18;
        g_GPIO_HOOK_DET = 12; 
		g_GPIO_JACK_IN_DET = 54;
		g_GPIO_LCD_TE = 0;
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
		g_GPIO_8M_WP = 2;
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
		g_GPIO_CAM_8M_RST_N = 10;
			 
	     msm_gpiomux_install(a60k_sr1_1_msm8960_gpio_configs,
			ARRAY_SIZE(a60k_sr1_1_msm8960_gpio_configs));					  
             break;			

	  // +++ ASUS_BSP : Add for A66 Proj	  	
          case A66_HW_ID_SR1_1:
          case A66_HW_ID_SR2:
          case A66_HW_ID_ER1:
             printk("A66 gpio config table = SR1_1\n");
        g_GPIO_HDMI_5V_ENABLE = 63;//Mickey+++
		g_GPIO_P01_I2C_LS_OE = 14;//Mickey+++
		g_GPIO_MIC2_BIAS_EN = 71;
        g_GPIO_SPK_AMP_EN = 2;
        g_GPIO_HS_PATH_EN = 47; 
        g_GPIO_HOOK_DET = 52;
		g_GPIO_JACK_IN_DET = 54;
		g_GPIO_LCD_TE = 0;
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
		g_GPIO_8M_WP = 48;
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
		g_GPIO_CAM_8M_RST_N = 10;
			 
	     msm_gpiomux_install(a66_sr1_msm8960_gpio_configs,
			ARRAY_SIZE(a66_sr1_msm8960_gpio_configs));					  
             break;	
	  // --- ASUS_BSP : Add for A66 Proj

          case A66_HW_ID_ER2:
	  case A66_HW_ID_ER3:
          case A66_HW_ID_PR:
                printk("A66 gpio config table = ER2, %x\n",g_A60K_hwID);
                g_GPIO_HDMI_5V_ENABLE = 63;//Mickey+++
                g_GPIO_P01_I2C_LS_OE = 14;//Mickey+++
                g_GPIO_MIC2_BIAS_EN = 71;
                g_GPIO_SPK_AMP_EN = 2;
                g_GPIO_HS_PATH_EN = 47; 
                g_GPIO_HOOK_DET = 52;
                g_GPIO_JACK_IN_DET = 54;
                g_GPIO_LCD_TE = 0;
//ASUS_BSP +++ Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
                g_GPIO_8M_WP = 48;
//ASUS_BSP --- Stimber "[A60K][8M][NA][Others]Full porting for 8M camera with ISP"
                g_GPIO_CAM_8M_RST_N = 10;

                msm_gpiomux_install(a66_er2_msm8960_gpio_configs,
                ARRAY_SIZE(a66_er2_msm8960_gpio_configs));

//+++ASUS_BSP Peter_lu : for set charging current to 1A.........................................
#ifdef CONFIG_MAXIM_8934_CHARGER
		if(ac_in)	{
			msm_gpiomux_install(a66_msm8960_charger_high_configs,
                	ARRAY_SIZE(a66_msm8960_charger_high_configs));
		}	else		{
			msm_gpiomux_install(a66_msm8960_charger_low_configs,
                	ARRAY_SIZE(a66_msm8960_charger_low_configs));
		}
#endif
//---ASUS_BSP Peter_lu : for set charging current to 1A..........................................			

				
             break;

          default:				  	
             printk(KERN_ERR "[ERROR] There is NO valid hardware ID\n");
             printk(KERN_ERR "[ERROR] Use the default one SR1_1 config table\n");
        g_GPIO_HDMI_5V_ENABLE = 63;//Mickey+++
		g_GPIO_P01_I2C_LS_OE = 14;
		g_GPIO_MIC2_BIAS_EN = 19;
        g_GPIO_SPK_AMP_EN = 2;
        g_GPIO_HS_PATH_EN = 47;
        g_GPIO_HOOK_DET = 52;
		g_GPIO_JACK_IN_DET = 54;
		g_GPIO_LCD_TE = 0;
		g_GPIO_8M_WP = 48;
		g_GPIO_CAM_8M_RST_N = 10;			 
	     msm_gpiomux_install(a60k_sr1_1_msm8960_gpio_configs,
			ARRAY_SIZE(a60k_sr1_1_msm8960_gpio_configs));						 
         break;
   }
   return 0;
}



