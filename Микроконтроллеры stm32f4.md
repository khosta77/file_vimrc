# Микроконтроллеры stm32f4

Я использую два микроконтроллера stm32f407vg и stm32f411ve.

![stm32f411](./img/stm32/block_diagram_stm32f411.png)

![stm32f407](./img/stm32/block_diagram_stm32f407.png)



# Общие моменты

## GPIO

![](./img/stm32/Basic_structure_of_a_five_volt_tolerant_GPIO.png)



```C
// 1. Включение тактирования
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOxEN;

// 2. Настроить перефирию
// Вывод
GPIOx->MODER |= (0x1 << (2 * 'pin'));  //GPIO_MODER_MODERx_0;
GPIOx->ODR |= (0x1 << 'pin');  //GPIO_ODR_ODx;  // записать 1
GPIOx->ODR &= ~GPIO_ODR_ODx;                    // записать 0
// Ввод
GPIOx->IDR |= (0x1 << 'pin');  //GPIO_IDR_IDx;
T temp = (((GPIOx->IDR) >> 'pin') & 0x00000001);  // считать значение
```

## TIM

Таймер в микроконтроллере это счетчик, который как только досчитывает до заданного временного значения генерирует прерывание. Счетчик находится отдельно от ядра и не использует его ресурсы, в отличии от прерывания. 

**Формула из лекций Денис Алексеевича:**
$$ t = (\text{ARR} \cdot \text{PSC})\frac{1}{F_{sys}} $$
Где *F_sys*  период микроконтроллера. в результате:

$$t \text{ }[\text{секунд}] = (\text{ARR} \cdot \text{PSC})\frac{10^{-7}}{8}$$

Частота тактов TIMx зависит от частоты шины APB. [Источник](http://microsin.net/programming/arm/an4776-general-purpose-timer-cookbook.html)

### TIM6 и TIM7

![](./img/stm32/Basic_timer_block_diagram_TIM6_TIM7.png)
Это считается самым простым таймером, он есть в stm32f407vg, но почему-то его нет в stm32f411ve.

Инициализация:
```C
// 1. Включаем тактирование
RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
// 2. Настраиваем время по формуле
TIM6->ARR = 4000;
TIM6->PSC = 500;
```
$$ t = (\text{ARR} \cdot \text{PSC})\frac{1}{F_{sys}} $$

```C
// 3. Включение прерываний
TIM6->DIER |= TIM_DIER_UIE;           //Разрешения прерывание от таймера
NVIC_EnableIRQ(TIM6_DAC_IRQn);        //Разрешение TIM6_DAC_IRQn прерывания
NVIC_SetPriority(TIM6_DAC_IRQn, 2);
// 4. Включение таймера
TIM6->CR1 |= TIM_CR1_CEN;
```

По аналогии инициализация работает и для *TIM7* меняем *TIM6* на *TIM7*, для другого таймера.

Прерывания:
```C
void TIM6_DAC_IRQHandler(void) {
	TIM6->SR &= ~TIM_SR_UIF;  //Сбрасываем флаг прерывания
	// TIM6->CR1 &= ~TIM_CR1_CEN;  // Соответсвенно в прирывании можно так же \
								   // отключить таймер
	// Наш код что то делает.
	// Тут не должно быть много кода, действий и т д
    // ...
}
```

Источники:
* Лекции
* [Статья 1](https://cxem.net/mc/mc250.php)

### TIM2 to TIM5

![](./img/stm32/General_purpose_timer_block_diagram_TIM2toTIM5.png)
Инициализируются точно так же, есть тонкости работы **(31.05.23 - с ними не сталкивался)**

### TIM1 and TIM8

![](./img/stm32/Advanced_control_timer_block_diagram_TIM1_TIM8.png)

**(01.06.23)** Детально не разбирал, но думаю логика такая же как +- с остальными, просто добавляются новые функции.

### TIM9 to TIM14

![](./img/stm32/General_purpos_timer_block_diagram_TIM9_a_TIM12.png)
![](./img/stm32/General_purpose_timer_block_diagram_TIM10_11_13_14.png =100x0)
## flash

**05.06.23** поговорим позже

## USART



## DMA

### USART + DMA

#### Полезные ссылки:
- [ ] [Репозиторий с реализацией DMA через CMSIS.](https://github.com/rmkeyser11/Engs62_Final/blob/master/DMA.c)
- [ ] [DMA + USART на CMSIS, но на С++](https://github.com/MCLEANS/STM32F4-USART-DMA-CONFIGURATION/blob/master/IMPLEMENTATION/src/main.cpp)
- [ ] [ADC + USART + DMA, на использует атрибуты еще какие то](https://github.com/mattmcf/ARM-microprocessor-WiFi-project/blob/master/ADC.c)
- [X] [Рабочий вариант, данные перекидыватся, USART + DMA](https://github.com/PavelSchal/usart_dma_cmsis_stm32f407ve/blob/main/src_inc_extrahiert/dma.c)
- [ ] [Еще один пример, нет задержок](https://github.com/ezydoez-ezrahogori/STM32F4_UART_TX_DMA/blob/main/20_uart_tx_dma/Src/uart.c)
- [ ] [Опять на С++ пример](https://github.com/MCLEANS/STM32F4-USART-DMA-CONFIGURATION/blob/master/IMPLEMENTATION/src/USART.cpp)
- [x] [Для ADC + DMA работает, но не USART](https://github.com/Saileshmurali/Register-Level-ADC-DMA-STM32F4-DISC1/blob/main/main.c)
- [x] [Шляпа какая-то](https://github.com/erenkeskin/STM32F4-Examples-with-Register/blob/master/STM32F4_USART/USART.c)

## ADC

## SPI

## I2C

## DAC

## DCMI

## LTDC

 








