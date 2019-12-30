#include "aos/hal/gpio.h"
#include "wm_gpio.h"
#include <stddef.h>
static const u8_t int_edge_map[] =
    {
        [IRQ_TRIGGER_RISING_EDGE] = WM_GPIO_IRQ_TRIG_RISING_EDGE,
        [IRQ_TRIGGER_FALLING_EDGE] = WM_GPIO_IRQ_TRIG_FALLING_EDGE,
        [IRQ_TRIGGER_BOTH_EDGES] = WM_GPIO_IRQ_TRIG_DOUBLE_EDGE,
};

int32_t hal_gpio_init(gpio_dev_t *gpio)
{
    if (gpio->port == NULL)
    {
        return -1;
    }

    u8_t gpio_dir = 0;
    u8_t gpio_attr = 0;

    switch (gpio->config)
    {
    /* 管脚用作功能引脚，如用于pwm输出，uart的输入引脚 */
    case ANALOG_MODE:
        break;
        /* 中断模式，配置为中断源 */
    case IRQ_MODE:
        gpio_dir = WM_GPIO_DIR_INPUT;
        gpio_attr = WM_GPIO_ATTR_FLOATING;
        break;
        /* 输入模式，内部包含一个上拉电阻 */
    case INPUT_PULL_UP:
        gpio_dir = WM_GPIO_DIR_INPUT;
        gpio_attr = WM_GPIO_ATTR_PULLHIGH;
        break;
        /* 输入模式，内部包含一个下拉电阻 */
    case INPUT_PULL_DOWN:
        gpio_dir = WM_GPIO_DIR_INPUT;
        gpio_attr = WM_GPIO_ATTR_PULLLOW;
        break;
        /* 输入模式，内部为高阻模式 */
    case INPUT_HIGH_IMPEDANCE:
        return -1; //W600没有高阻模式
        /* 输出模式，普通模式 */
    case OUTPUT_PUSH_PULL:
        gpio_dir = WM_GPIO_DIR_OUTPUT;
        gpio_attr = WM_GPIO_ATTR_FLOATING;
        break;
        /* 输出模式，输出高电平时，内部为高阻状态 */
    case OUTPUT_OPEN_DRAIN_PULL_UP:
        gpio_dir = WM_GPIO_DIR_OUTPUT;
        gpio_attr = WM_GPIO_ATTR_PULLLOW;
        break;
        /* 输出模式，输出高电平时，被内部电阻拉高 */
    case OUTPUT_OPEN_DRAIN_NO_PULL:
        gpio_dir = WM_GPIO_DIR_OUTPUT;
        gpio_attr = WM_GPIO_ATTR_PULLHIGH;
        break;
    default:
        break;
    }
    tls_gpio_cfg(gpio->port, gpio_dir, gpio_attr);
    return 0;
}

// 使指定GPIO输出高电平
int32_t hal_gpio_output_high(gpio_dev_t *gpio)
{
    tls_gpio_write(gpio->port, 1);

    return 0;
}
//使指定GPIO输出低电平
int32_t hal_gpio_output_low(gpio_dev_t *gpio)
{
    tls_gpio_write(gpio->port, 0);

    return 0;
}
//使指定GPIO输出翻转
int32_t hal_gpio_output_toggle(gpio_dev_t *gpio)
{
    u8_t ret = 0;
    ret = tls_gpio_read(gpio->port);
    tls_gpio_write(gpio->port, !ret);

    return 0;
}
//获取指定GPIO管脚的输入值
int32_t hal_gpio_input_get(gpio_dev_t *gpio, uint32_t *value)
{
    *value = tls_gpio_read(gpio->port);

    return 0;
}
//使能指定GPIO的中断模式，挂载中断服务函数
int32_t hal_gpio_enable_irq(gpio_dev_t *gpio, gpio_irq_trigger_t trigger,
                            gpio_irq_handler_t handler, void *arg)
{

    tls_gpio_isr_register(gpio->port, handler, arg);
    tls_gpio_irq_enable(gpio->port, int_edge_map[trigger]);

    return 0;
}
//关闭指定GPIO的中断
int32_t hal_gpio_disable_irq(gpio_dev_t *gpio)
{
    tls_gpio_irq_disable(gpio->port);

    return 0;
}
//清除指定GPIO的中断状态
int32_t hal_gpio_clear_irq(gpio_dev_t *gpio)
{
    tls_clr_gpio_irq_status(gpio->port);
    return 0;
}
//	关闭指定GPIO
int32_t hal_gpio_finalize(gpio_dev_t *gpio)
{
    return -1;//SDK库没有相关函数直接返回错误
}