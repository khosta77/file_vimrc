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

## EXTI

Внешние прерывания. Прежде всего необходимо разобраться с тем, что такое прерывания.  
В МК Cortex-M есть два понятия, которые часто путают *Interrupt* и *Event*.  
*Event* — это событие (аппаратное или программное), на которое могут реагировать ядро или периферийные блоки. Одним из вариантов реакции может быть — прерывание.  
*Interrupt* — это прерывание работы программы и переход управления в специализированный участок обработчик прерывания. 
  
Взаимосвязь между *Event* и *Interrupt* заключается в следующем:  
*Каждый Interrupt вызывается Event, но не каждый Event вызывает Interrupt.*   
Помимо прерываний, события могут активировать и другие возможности МК.

```C
//// Настройка порта прерывания
// 1. Включение тактрования
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOxEN;
// 2. Настройка на чтение
GPIOx->MODER &= ~(0x03 << (2 * PIN));
// 3. Настраиваем работу на понижение или повышение ввода(в примере понижение)
GPIOx->PUPDR |= (0x1 << (2 * PIN + 1));

//// Настраиваем прерывание
// 1. Снимите маску с прерывания
EXTI->IMR |= (1 << PIN);
// 2. Установите, будет ли опускающаяся кромка или поднимающаяся кромка(опускающаяся)
EXTI->FTSR |= (1 << PIN);
// 3. Разрешение прерывания
EXTI->PR |= (1 << PIN);

//// SYSCFG
// 1. Включаем тактирование SYSC
RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
// 2. Настраиваем Регистр конфигурации внешнего прерывания SYSCFG пина
SYSCFG->EXTICR[y] &= ~(0xF << PIN);
// (12.06.23) - эти точки детально смотреть в документации

//// Включаем прерывания
NVIC_EnableIRQ('EXTI1_IRQn');
```

Прерывания делятся на группы:
* Пины: *0*         NVIC: **EXTI0_IRQn**
* Пины: *1*         NVIC: **EXTI1_IRQn**
* Пины: *2*        NVIC: **EXTI2_IRQn**
* Пины: *3*        NVIC: **EXTI3_IRQn**
* Пины: *4*        NVIC: **EXTI4_IRQn**
* Пины: *5-9*    NVIC: **EXTI9_5_IRQn**
* Пины: *10-15* NVIC: **EXTI15_10_IRQn**
На stm32f411ve нету всех эти прерываний.
Структура прерывания:
```C
void EXTI'1'_IRQHandler(void){
	if((EXTI->PR & EXTI_PR_PRx) != 0x00){
		// DO SOMTHING...
		EXTI->PR |= EXTI_PR_PRx;
	}
}
```

**Источники:**
* [нормальный чувак](https://github.com/sss22213/stm32f4_exti0_experience/blob/master/blink.c)

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

**Источники:**
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
![](./img/stm32/General_purpose_timer_block_diagram_TIM10_11_13_14.png)
### ШИМ

**Широтно-импульсная модуляция** (ШИМ, pulse-width modulation (PWM)) — процесс управления мощностью методом пульсирующего включения и выключения потребителя энергии.

![](./img/stm32/PWM.png)

Сигнал ШИМ характеризуется скважностью **S** или её обратно величиной - коэффициент заполнения **D**. 
$$S = \frac{T}{\tau} = \frac{1}{D}$$
Будет полезно вспомнить формулу для периода таймера:
$$ t = (\text{ARR} \cdot \text{PSC})\frac{1}{F_{sys}} $$
Пример плавного повышения яркости светодиода:
```C
#define PIN 12  // Номер пина
#define S 1     // длительности импульса в циклах, по умолчанию

/*  Прерывание надо для плавного увеличения ШИМ, но в целом можно и без него,
 *  а рализовать отдельными функциями. В данном случае, светодиод не будет мигать,
 *  а будет плавно загоратся.
 * */
void TIM4_IRQHandler(void) {
    // 0. Снимаем маркер прервыания
	TIM4->SR &= ~TIM_SR_UIF;
	
    // 1. Выключаем таймер для его редактирования
	TIM4->CR1 &= ~TIM_CR1_CEN;
	
    // 2. Увеличиваем длительность импульса
    if (TIM4->CCR1 != 200)
        TIM4->CCR1 += 1;
    else
        TIM4->CCR1 = S;
        
    // 3. Запускаем таймер
    TIM4->CR1 |= TIM_CR1_CEN;
}

int main(void) {
    //// Настраиваем порт светодиода
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    
    // 0. ШИМ назначается на альтернативную функцию
    GPIOD->MODER |= (0x2 << (2 * PIN));
    GPIOD->AFR[1] |= (0x2 << 16); 
    
    //// Настравиваем ШИМ на таймере
    // 0. включаем тактирование нужного таймера
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // 1. Настраиваем период срабатывания таймера
    TIM4->PSC = 3000;
    TIM4->ARR = 100;
    
    ////  Настраиваем "захват/сравнение
    // 2. Тут включается PWM1, строка OC1M"
    TIM4->CCMR1 |= 0x60;
    
    // 3. Установка длительности импульса в циклах
    TIM4->CCR1 = S;
    
    // 4. Включение CC1
    TIM4->CCER |= 0x1;
    
    // *. Включаем прерывание
    TIM4->DIER |= TIM_DIER_UIE;    
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_SetPriority(TIM4_IRQn, 2);
    
    // 5. Запускаем таймер
    TIM4->CR1 |= TIM_CR1_CEN;
    
    while(1);
}
```

**Источники:**
* [Формула скважности](https://yarllo.ru/wp-content/uploads/3/a/e/3ae4a94bc1267fe8774fdb8bfcbd92de.jpeg)
* [Альтернативные функции на пинах GPIO](http://microsin.net/programming/arm/stm32f407-gpio-pins-alternate-function.html)
* [Хороший человек с github](https://github.com/jrsa/stm32f4_pwm/blob/master/main.c)
* [Ещё один пример использования](https://github.com/acakbudak/stm32f4_pwm/blob/master/stm32f44re_pwm.c)

[//]: [Остановился, при поиске](https://github.com/search?q=stm32f4_PWM&type=repositories&p=6)

## flash
Flash память это внутренняя память микроконтроллера, которая сохраняет свое значение после обесточивания микроконтроллера, тем самым мы можем сохранять какие-нибудь значения.
![](./img/stm32/FLASH_block_diagramm.png)
В stm32f40x и stm32f41x память разделена на 12 секторов.
![](./img/stm32/FLASH_module_organization.png)
### Инициализация
Чтобы начать работу со flash ее необходимо проинициализировать. В ней используются временные задержки, таблица которых приведена ниже.
![](./img/stm32/FLASH_number_of_wiat_states.png)
```C
void stp_flash_init() {
	//// Конфигурация FLASH
	// 1. Очистка флагов
    FLASH->ACR = 0;
	
    // 2. Установка задержки (5 секунд)
	FLASH->ACR |= FLASH_ACR_LATENCY_5WS;  // (5 <<  0);
	
    // 3. Включаем кэш команд
    FLASH->ACR |= FLASH_ACR_ICEN;  //(1 <<  9);
	
    // 4. Включаем кэш данных
    FLASH->ACR |= FLASH_ACR_DCEN;  //(1 << 10);
	
    // 5. PLL Выбран в качестве системных часов \
    // если не работает, убрать макрос.
    RCC->CFGR |= RCC_CFGR_SW_1;  //(2 <<  0);
	
    // 6. Ждем пока PLL включится
	while ((RCC->CFGR & 0x0F) != 0x0A);
}
```

### Разблокировка и блокировка доступа к памяти
Память защищена блокиратором и для того, чтобы иметь к ней доступ ее необходимо разблокировать, это возможно сделать спомощью записи двух ключей в регистр.
```C
void stp_flash_unlock() {
    //// Разблокировка памяти
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = 0x45670123;
        FLASH->KEYR = 0xCDEF89AB;
    }
}
```
После работы доступ к памяти следует заблокировать.
```C
void stp_flash_lock() {
    //// Блокировка памяти
    FLASH->CR |= FLASH_CR_LOCK;
}
```
#### Уточнение
Так же после того как мы разблокировали память, стоит подождать пока она разблокируется.
```C
#define STP_FLASH_WHILE_BSY()       { while (FLASH->SR & FLASH_SR_BSY){} }
```
### Запись данных
```C
static void stp_flash_write(uint32_t address, uint8_t data) {
    //// Запись данных в одну клетку
    // 1. Разрешаем программирование
    FLASH->CR |= FLASH_CR_PG;

    // 2. Записываем в ячейку данных один байт
    *(uint8_t *)address = data;

    // 3. Ждем пока запись завершится
    STP_FLASH_WHILE_BSY();
    
    // 4. Запрещаем программирование
    FLASH->CR &= ~FLASH_CR_PG;
}

static void stp_flash_write_buffer(uint32_t address, const uint8_t * dataBuffer, uint32_t size) {
    //// Запись данных в одну ячейку
	// 1. Разблокируем память для записи данных
    stp_flash_unlock();
    
    // 2. Подождем пока произойдет разблокировка
    STP_FLASH_WHILE_BSY();

    // 3. Цикл записи
    while (size >= sizeof(uint8_t)) {
    	stp_flash_write(address, *(const uint8_t *)dataBuffer);
        address += sizeof(uint8_t);
        dataBuffer += sizeof(uint8_t);
        size -= sizeof(uint8_t);
    }

    // 4. Заблокируем память. Признак хорошего тона
    stp_flash_lock();
}

// Запись данных во flash
void stp_flash_write_8(uint32_t address, const uint8_t *dataBuffer, uint32_t size) {
	stp_flash_write_buffer(address, dataBuffer, size);
}

void stp_flash_write_16(uint32_t address, const uint16_t *dataBuffer, uint32_t size) {
    uint8_t buffer[(sizeof(uint16_t) * size)];
    memcpy(buffer, dataBuffer, (sizeof(uint16_t) * size));
    stp_flash_write_buffer(address, buffer, (sizeof(uint16_t) * size));
}

void stp_flash_write_32(uint32_t address, const uint32_t *dataBuffer, uint32_t size) {
	uint8_t buffer[(sizeof(uint32_t) * size)];
	memcpy(buffer, dataBuffer, (sizeof(uint32_t) * size));
	stp_flash_write_buffer(address, buffer, (sizeof(uint32_t) * size));
}
```

### Чтение данных
```C
// Чтение данных из flash
void stp_flash_read_8(uint32_t address, uint8_t *buffer, uint32_t size) {
    memcpy(buffer, (uint8_t*) address, size);
}

void stp_flash_read_16(uint32_t address, uint16_t *buffer, uint32_t size) {
	uint8_t char_buffer[(sizeof(uint16_t) * size)];
	memcpy(char_buffer, (uint8_t*) address, (sizeof(uint16_t) * size));
	memcpy(buffer, char_buffer, (size * sizeof(uint16_t)));
}

void stp_flash_read_32(uint32_t address, uint32_t *buffer, uint32_t size) {
	uint8_t char_buffer[(sizeof(uint32_t) * size)];
	memcpy(char_buffer, (uint8_t*) address, (sizeof(uint32_t) * size));
	memcpy(buffer, char_buffer, (size * sizeof(uint32_t)));
}
```

### Адрес памяти

```C
#define START_ADDRESS 	0x0800C000 	//	(FLASH_BASE + 48*1024)
```

### Пример использования

```C
stp_flash_init();
stp_flash_erase(3);
const uint32_t SIZE = 254;
uint32|16|8_t arr1[SIZE];
for (uint32_t i = 0; i < SIZE; ++i)
    arr1[i] = (i + 1);


stp_flash_write_32|16|8(START_ADDRESS, arr1, STP_SIZE_OF_ARRAY(arr1));
uint32|16|8_t arr2[SIZE];
stp_flash_read_32|16|8(START_ADDRESS, arr2, STP_SIZE_OF_ARRAY(arr2));
for (uint32_t i = 0; i < SIZE; i++) {
	trace_printf("error %u %u %u\n", i, *(arr2 + i), *(arr1 + i));
...
stp_flash_lock();
```

**Источники:**
* [Статья](http://microsin.net/programming/arm/stm32f4-embedded-flash-memory-interface.html)
* [Хороший человек с github](https://github.com/erenkeskin/STM32F4-Examples-with-Register/blob/master/STM32F4_FLASH_Bootloader/flash.c)
* Документация

## USART

Универсальный синхронный/асинхронный трансивер (universal synchronous asynchronous receiver transmitter, **USART**) - протокол обмена данными. Любой двунаправленный обмен USART требует как минимум двух сигнальных выводов: входные принимаемые данные (Receive Data In, RX) и выходные передаваемые данные (Transmit Data Out, TX):

**RX**: вход последовательных принимаемых данных. Используются техники передискретизации для восстановления данных, чтобы отделить полезные приходящие данные от шума.

**TX**: выход передаваемых данных. Когда передатчик запрещен, ножка выхода возвратит свою конфигурацию порта ввода/вывода (GPIO). Когда передатчик разрешен и ничего не передается, уровень выхода ножки TX находится в лог. 1. В режимах single-wire и smartcard, этот вывод I/O используется и для передачи, и для приема данных (на уровне USART данные затем принимаются на SW_RX).
[//]: ![USART](./img/stm32/USART_block_diagram.png)
![](/img/stm32/USART_block_diagram_ru.png)
Через эти выводы последовательные данные принимаются и передаются в нормальном режиме USART как фреймы. В этом процессе используется следующее:

• Состояние ожидания линии (Idle Line) до передачи или приема.  
• Start-бит.  
• Слово данных (8 или 9 бит), самый младший бит слова (LSB) идет первым.  
• 0.5,1, 1.5, 2 Stop-стоп-бит, показывающих завершение фрейма.  
• Используется дробный генератор скорости - с 12-разрядной мантиссой и 4-битной дробной частью.  
• Регистр статуса (USART_SR).  
• Регистр данных (USART_DR).  
• Регистр генератора скорости (USART_BRR) с 12-разрядной мантиссой и 4-битной дробной частью.  
• Регистр защитного времени, Guardtime Register (USART_GTPR) в случае использования режима Smartcard.
Пример временных диаграмм:
[//]: ![](/img/stm32/USART_word_length_programming.png)
![](/img/stm32/USART_word_length_programming_ru.png)

**Дробный генератор скорости**. Скорость обмена (baud rate) для приемника и передатчика (обоих сигналов RX и TX) устанавливается в одинаковое значение, программируемое коэффициентами Mantissa и Fraction делителя USARTDIV.

Формула 1. скорость для стандартного USART (включая режим SPI):
$$\text{Tx/Rx baud} = \frac{f_{CK}}{8 \cdot (2 - OVER8)\cdot USARTDIV}$$
Формула 2. скорость для режимов Smartcard, LIN и IrDA:
$$\text{Tx/Rx baud} = \frac{f_{CK}}{16 \cdot USARTDIV}$$
USARTDIV это число с фиксированной запятой без знака, закодированное в регистре USART_BRR.
![](./img/stm32/USART_BR.png)

Пины на stm32f4:

![](./img/stm32/Figure_UART_PinPack.png)

### Простая инициализация

```C
#define CPU_CLOCK SystemCoreClock
#define MY_BDR 115200

#define MYBRR (CPU_CLOCK / (16 * MY_BDR))

void GPIO_init() {
	// 1. Вкл. тактирование
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	// 2. назначили пины
	GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);
	// Альтернативные функции
	GPIOA->AFR[0] |= 0x77 << 8;
}

void USART_init() {
	// 1. Вкл тактирование
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// 2. Задали частоту работы
	USART2->BRR = MYBRR;
	// 3. Настроили на чтение и запись
	USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
	// 4. Установили STOP бит
	USART2->CR2 |= (USART_CR2_STOP_1);
	// 5. Вкл USART
	USART2->CR1 |= USART_CR1_UE;
}

/* Чтение */
uint8_t readUSART() {
	uint8_t test_usart2_sr_1 = USART2->SR;
    if ((USART2->SR & USART_SR_RXNE) == USART_SR_RXNE) {
    	return USART2->DR;
    }
    return -1;
}

/* Запись */
uint8_t writeUSART(uint8_t data) {
	uint8_t test_usart2_sr_2 = USART2->SR;
    if ((USART2->SR & USART_SR_TXE) == USART_SR_TXE) {
    	USART2->DR = data;
    	return 0;
    }
    return -1;
}

int main(void) {
	SystemCoreClockUpdate();
	GPIO_init();
	USART_init();
	uint8_t temp = 0x21;
	while(1) {
		writeUSART(temp);
		for (int i = 0; i  < 50; i++);
		uint8_t temp_read = readUSART();
		// Вывод данных
    	trace_printf("temp_read: %u\n", temp_read);
		if (temp_read != 255) {
			temp = ++temp % 254;
		}
	}
}
```

### USART + DMA

Просто USART не рационально использовать, так как можно потерять данные при передаче или долго их ожидать. Стоит использовать связку DMA + USART.

```C
#define CPU_CLOCK SystemCoreClock
#define MY_BDR 115200

#define MYBRR (CPU_CLOCK / (16 * MY_BDR))

void GPIO_init() {
    // 0. Вкл. тактирование
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // 1. Назначили пины на альтернативный режим работы
    GPIOA->MODER |= (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1);
    // 2. Альтернативные функции
    GPIOA->AFR[0] |= (0x77 << 8);
    GPIOA->AFR[1] |= (0x77 << 4);

    // Включаем светодиоды
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= (GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0);
}

void USART_init() {
    //// включаем USART2 PA2, PA3
    // 0. Вкл тактирование
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    // 1. Задали частоту работы
    USART2->BRR = MYBRR;
    // 2. Настроили на чтение и запись
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_RE);
    // 3. Установили STOP бит
    USART2->CR2 |= (USART_CR2_STOP_1);
    // 4. Включили DMA для USART
    USART2->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    // 5. Вкл USART
    USART2->CR1 |= USART_CR1_UE;

    //// Включаем USART1 PA9, PA10
    // Аналогично, как и для USART2, только USART2->USART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = MYBRR;
    USART1->CR1 |= (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE);
    USART1->CR2 |= (USART_CR2_STOP_1);
    USART1->CR3 |= (USART_CR3_DMAR | USART_CR3_DMAT);
    USART1->CR1 |= USART_CR1_UE;
}

#define SIZE 128

uint8_t usart1_receive_buffer[(2 * SIZE)];
uint8_t usart2_receive_buffer[(2 * SIZE)];

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {
        GPIOD->ODR ^= GPIO_ODR_OD12;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }
}

void DMA2_Stream2_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF2) == DMA_LISR_TCIF2) {
        GPIOD->ODR ^= GPIO_ODR_OD13;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
    }
}

void DMA_init() {
    // DMA1_Stream6 это USART2_TX
    // DMA2_Stream2 это USART1_RX

    // 0. Включили тактирование DMA
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // 1. Выбираем канал
    DMA1_Stream6->CR |= (0x4 << 25);  // Канал 4
    DMA2_Stream2->CR |= (0x4 << 25);  // Канал 4
    
    // 2. Устанавливаем размер ячейки (8-бит(можно 16-бит, 32-бит))
    DMA1_Stream6->CR &= ~DMA_SxCR_MSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_MSIZE;

    // 3. Установите размер периферийных данных равным 8 битам
    DMA1_Stream6->CR &= ~DMA_SxCR_PSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_PSIZE;

    // 4. Установите режим увеличения объема памяти
    DMA1_Stream6->CR |= DMA_SxCR_MINC;
    DMA2_Stream2->CR |= DMA_SxCR_MINC;

    // 5. Включаем режим работы
    DMA1_Stream6->CR |= (0x01<<6);  // Из памяти в перефирию
    DMA2_Stream2->CR &= ~(3UL<<6);  // Из переферии в память

    // 6. Включить циклическую запись
    DMA1_Stream6->CR |= DMA_SxCR_CIRC;
    DMA2_Stream2->CR |= DMA_SxCR_CIRC;

    // 7. Включаем прерывания
    DMA1_Stream6->CR |= DMA_SxCR_TCIE;
    DMA2_Stream2->CR |= DMA_SxCR_TCIE;

    // 8. Количество элементов данных, подлежащих передаче	
    DMA1_Stream6->NDTR = SIZE;
    DMA2_Stream2->NDTR = SIZE;

    // 9. Задаем адрес переферии
    DMA1_Stream6->PAR = (uint32_t)(&USART2->DR);
    DMA2_Stream2->PAR = (uint32_t)(&USART1->DR);

    // 10. Задаем адрес памяти
    DMA1_Stream6->M0AR = (uint32_t)&usart2_receive_buffer[0];
    DMA2_Stream2->M0AR = (uint32_t)&usart1_receive_buffer[0];

    // 11. Настройка прерываний
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 5);

    NVIC_EnableIRQ(DMA2_Stream2_IRQn);
    NVIC_SetPriority(DMA2_Stream2_IRQn, 4);

    // 12. Включаем DMA
    DMA1_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
    // Включение DMA происходит только после того, как мы инициализировали GPIO и USART
    // И не надо его выключать и включать после, он уже работает в цикле.
}

void USART_DMA_init() {
    SystemCoreClockUpdate();
    GPIO_init();
    USART_init();
    DMA_init();
}

void write_dma(uint8_t *df, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *(usart2_receive_buffer + i) = *(df + i);
    }
}

void read_dma(uint8_t *df, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        *(df + i) = *(usart1_receive_buffer + i);
    }
}
```

* [Рабочий вариант](https://github.com/khosta77/stm32f4_USART_DMA_CMSIS/blob/main/src/main.c)

## DMA
Прямой доступ к памяти (от англ. *Direct Memory Access*, *DMA*) – технология прямого доступа к памяти, минуя центральный процессор. Вся ее суть заключается в том, чтобы по команде от периферии или ядра взять и скопировать кусок памяти с одного места на другой.

Структура одного блока DMA, в stm32f4 их 2:
![](./img/stm32/DMA_block_diagram.png)

Взаимодействие блоков DMA:

![](./img/stm32/DMAs.png)

Один блок DMA имеет 8 потоков(Stream), а каждый поток имеет 8 каналов(Channel). Активных потоков может быть, хоть все 8, но канал на них выбран только 1. Таблица с даннными о потоках и каналах:
![](./img/stm32/DMA_DMA1_request_mapping_ch1.png)
![](DMA_DMA1_request_mapping_ch2.png)
![](./img/stm32/DMA_DMA2_request_mapping.png)
*P.s. вот эта таблица для stm32f40x, stm32f41x и stm32f42xx, stm32f43xx - ну для них просто добавляются некоторые дополнительные данные.*

Что касается взаимодействия DMA с периферией, то там где это уместно приведен пример использования DMA.

**Источники:**
* [Переведенный на русский язык Datasheat для stm32f429](https://arm-stm.blogspot.com/2016/10/stm32f-adc-with-dma-on-cmsis.html)
* [Переведенный на русский язык Datasheat для USART](http://microsin.net/programming/arm/stm32f4xx-uart-and-usart.html)

#### Полезные ссылки:
- [x] [DMA + USART на CMSIS, но на С++](https://github.com/MCLEANS/STM32F4-USART-DMA-CONFIGURATION/blob/master/IMPLEMENTATION/src/main.cpp)
- [x] [ADC + USART + DMA, на использует атрибуты еще какие то](https://github.com/mattmcf/ARM-microprocessor-WiFi-project/blob/master/ADC.c) **ШЛЯПА**
- [X] [Рабочий вариант, данные перекидыватся, USART + DMA](https://github.com/PavelSchal/usart_dma_cmsis_stm32f407ve/blob/main/src_inc_extrahiert/dma.c)
- [x] [Еще один пример, нет задержок](https://github.com/ezydoez-ezrahogori/STM32F4_UART_TX_DMA/blob/main/20_uart_tx_dma/Src/uart.c)**ШЛЯПА**
- [x] [Шляпа какая-то](https://github.com/erenkeskin/STM32F4-Examples-with-Register/blob/master/STM32F4_USART/USART.c)**ШЛЯПА**
* [x] [Денис, DMA + USART для F0](https://github.com/DenisOffor/USART_DMA_LEARNING/blob/main/UART_DMA_learning/src/main.c)
* [x] [Англиский сайт](https://arm-stm.blogspot.com/2016/10/stm32f-adc-with-dma-on-cmsis.html)*ХЗ*
**Ну последение сайты, которые смотрел**
* [x] [Почти работает](https://github.com/PavelSchal/usart_dma_cmsis_stm32f407ve/blob/main/src_inc_extrahiert/SysClock.c)
* [x] [С прерываниями](https://github.com/ezydoez-ezrahogori/STM32F4_UART_TX_DMA/blob/main/20_uart_tx_dma/Src/main.c)

## ADC
Аналого цифровой преобразователь (analog-to-digital converters, **ADC**) - устройство, преобразующее входной аналоговый сигнал в дискретный код (цифровой сигнал).
![](./img/stm32/ADC_plot.png)
Существует множество типов АЦП, однако в рамках данной статьи мы ограничимся рассмотрением только следующих типов:  
- АЦП параллельного преобразования (прямого преобразования, flash ADC)
- АЦП последовательного приближения (SAR ADC)
- Следящий АЦП

#### АЦП прямого преобразования
![](./img/stm32/ADC_parallel.png)
Принцип действия АЦП предельно прост: входной сигнал поступает одновременно на все «плюсовые» входы компараторов, а на «минусовые» подается ряд напряжений, получаемых из опорного путем деления резисторами R. Для схемы на рис. 1 этот ряд будет таким: (1/16, 3/16, 5/16, 7/16, 9/16, 11/16, 13/16) Uref, где Uref – опорное напряжение АЦП.  
  
Пусть на вход АЦП подается напряжение, равное 1/2 Uref. Тогда сработают первые 4 компаратора (если считать снизу), и на их выходах появятся логические единицы. Приоритетный шифратор (priority encoder) сформирует из «столбца» единиц двоичный код, который фиксируется выходным регистром.  
  
Теперь становятся понятны достоинства и недостатки такого преобразователя. Все компараторы работают параллельно, время задержки схемы равно времени задержки в одном компараторе плюс время задержки в шифраторе. Компаратор и шифратор можно сделать очень быстрыми, в итоге вся схема имеет очень высокое быстродействие.  
  
Но для получения N разрядов нужно 2^N компараторов (и сложность шифратора тоже растет как 2^N). Схема на рис. 1. содержит 8 компараторов и имеет 3 разряда, для получения 8 разрядов нужно уже 256 компараторов, для 10 разрядов – 1024 компаратора, для 24-битного АЦП их понадобилось бы свыше 16 млн. Однако таких высот техника еще не достигла.

#### АЦП последовательного приближения
Аналого-цифровой преобразователь последовательного приближения (SAR, Successive Approximation Register) измеряет величину входного сигнала, осуществляя ряд последовательных «взвешиваний», то есть сравнений величины входного напряжения с рядом величин, генерируемых следующим образом:  
  
1. на первом шаге на выходе встроенного цифро-аналогового преобразователя устанавливается величина, равная 1/2Uref (здесь и далее мы предполагаем, что сигнал находится в интервале (0 – Uref).  
  
2. если сигнал больше этой величины, то он сравнивается с напряжением, лежащим посередине оставшегося интервала, т.е., в данном случае, 3/4Uref. Если сигнал меньше установленного уровня, то следующее сравнение будет производиться с меньшей половиной оставшегося интервала (т.е. с уровнем 1/4Uref).  
  
3. Шаг 2 повторяется N раз. Таким образом, N сравнений («взвешиваний») порождает N бит результата.
![](./img/stm32/ADC_serial.png)
Таким образом, АЦП последовательного приближения состоит из следующих узлов:  
  
1. Компаратор. Он сравнивает входную величину и текущее значение «весового» напряжения (на рис. 2. обозначен треугольником).  
  
2. Цифро-аналоговый преобразователь (Digital to Analog Converter, DAC). Он генерирует «весовое» значение напряжения на основе поступающего на вход цифрового кода.  
  
3. Регистр последовательного приближения (Successive Approximation Register, SAR). Он осуществляет алгоритм последовательного приближения, генерируя текущее значение кода, подающегося на вход ЦАП. По его названию названа вся данная архитектура АЦП.  
  
4. Схема выборки-хранения (Sample/Hold, S/H). Для работы данного АЦП принципиально важно, чтобы входное напряжение сохраняло неизменную величину в течение всего цикла преобразования. Однако «реальные» сигналы имеют свойство изменяться во времени. Схема выборки-хранения «запоминает» текущее значение аналогового сигнала, и сохраняет его неизменным на протяжении всего цикла работы устройства.  
  
Достоинством устройства является относительно высокая скорость преобразования: время преобразования N-битного АЦП составляет N тактов. Точность преобразования ограничена точностью внутреннего ЦАП и может составлять 16-18 бит (сейчас стали появляться и 24-битные SAR ADC, например, AD7766 и AD7767).
#### Следящий АЦП
![](./img/stm32/ADC_watcher_1.png)


![](./img/stm32/ADC_watcher_2.png)
#### АЦП в stm32f4
АЦП ы stm32f4 представлен 3 блоками:
![](./img/stm32/ADC_block_1.png)
Эти же блоки в более крупном формате:
![](./img/stm32/ADC_block_2.png)
И подробно один блок:
![](./img/stm32/ADC_block_3.png)
Как люди пишут в переведенной документации там очень много всего, лучше на примерах делать.
```C
#define SIZE 2
uint32_t adc1[SIZE];

void GPIO_init() {
    //// Настройка GPIO
    // 0. Тактирование GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // 1. Настраиваем нужный порт на аналоговый режим работы
	GPIOA->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE1_1); 
}

void DMA2_Stream4_IRQHandler(void) {
	if((DMA2->HISR & (DMA_HISR_TCIF4 | DMA_HISR_HTIF4)) == (DMA_HISR_TCIF4 | DMA_HISR_HTIF4)) {
        // Очищаем регистры отвечающие за прерывание.
		DMA2->HIFCR = (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4);
	} 
}

void DMA_init() {
    // 0. Включаем DMA2
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // 1. По таблице выбираем канал 0
    // Так как канал ноль, то бит устанавливать не обезательно
//    DMA2_Stream4->CR &= ~(0x7 << 25);
    
    // 2. Задаем откуда, куда и сколько данных передавать
	DMA2_Stream4->PAR |= (uint32_t)&ADC1->DR;
	DMA2_Stream4->M0AR |= (uint32_t)&adc1;
	DMA2_Stream4->NDTR = SIZE;

    // 3. Настройка
    // 3.1 DMA_SxCR_CIRC - Включаем круговой режим работы
    // 3.2 DMA_SxCR_MINC - Режим увеличения объема памяти
    // 3.3 DMA_SxCR_PSIZE_1 - Длина 32-bit
    // 3.4 DMA_SxCR_MSIZE_1 - Длина 32-bit
    // 3.5 DMA_SxCR_PL_1 - Высокий уровень приоритета, не обезательная скорее всего настройка, 
    //                     добавить если несколько будеь
    // 3.6 DMA_SxCR_TCIE - Прерывания
    DMA2_Stream4->CR = (DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_PSIZE_1 | DMA_SxCR_MSIZE_1 | DMA_SxCR_TCIE);

    // 4. Прерывания
    NVIC_EnableIRQ(DMA2_Stream4_IRQn);
    NVIC_SetPriority(DMA2_Stream4_IRQn, 4);

    // 5. Запуск
	DMA2_Stream4->CR |= DMA_SxCR_EN;
}

void ADC_init(){
    // -1. Инициализировать GPIO
    GPIO_init();

    // 0. Включаем тактирование ADC1
    RCC->APB2ENR = RCC_APB2ENR_ADC1EN;

    // 1. Включаем режим сканирования
    ADC1->CR1 |= ADC_CR1_SCAN;

    // 2. Настройка
    // 2.1 ADC_CR2_ADON - Включение ADC
    // 2.2 ADC_CR2_CONT - Обработка в цикле
    // 2.3 ADC_CR2_EOCS - Завершение выбора преобразования, для DMA
    // 2.4 ADC_CR2_DMA - Работа с DMA
    // 2.5 ADC_CR2_DDS - Тоже для DMA что то
    ADC1->CR2 |= (ADC_CR2_ADON | ADC_CR2_CONT | ADC_CR2_EOCS | ADC_CR2_DMA | ADC_CR2_DDS);

    // 3. Назначаем пины с которых брать анлаоговый сигнал
	ADC1->SQR3 |= ((0 << 0) | (1 << 5));

    // 4. Колличество каналов с которых брать сигнал (L = (n - 1))
	ADC1->SQR1 |= (1 << 20);

    // 5. Включаем DMA
    DMA_init();

    // 6. Запускаем конвертацию
    ADC1->CR2 |= ADC_CR2_SWSTART;
}
```
*Есть проблема в коде выше, инициализация происходит последовательно с GPIO. То есть нельзя сделать так, чтобы через DMA данные записывались в массив не с нулевого GPIO.*

**Источники:**
* [Отличная статья на хабре](https://habr.com/ru/articles/125029/)
* [Дополнение к статье](https://baumanka.pashinin.com/IU1/sem10/Управляющие%20ЭВМ%20и%20комплексы%20(Суханов)/_4_2011.pdf)
* [Переведенная документация](http://microsin.net/programming/arm/stm32f429-analog-to-digital-converters.html)

#### Полезные ссылки:
- [ ] [ADC + DMA через CMSIS.](https://github.com/rmkeyser11/Engs62_Final/blob/master/DMA.c)
- [ ] [Простой пример использования ADC](https://github.com/lamer0k/Lections/blob/master/Lection6.adoc)
- [ ] [Для ADC + DMA работает](https://github.com/Saileshmurali/Register-Level-ADC-DMA-STM32F4-DISC1/blob/main/main.c)
- [ ] [Пример использования АЦП без ДМА](https://github.com/donRumata03/STM32F4-CMSIS-lessons/blob/master/ADC_Regular_Channels/ADC.c)

## DAC

**Цифро-Аналоговый преобразователь** - 12 разрядный, как и **ADC** в тонокости работы документация не углуюлялась и в данных момент (07.07.23) я тоже. Схема:

![](./img/stm32/DAC_block_diadram.png)

Нюанс в работу всплыл, что кроме *PA5* и *PA4* нельзя настроить аналоговый канал.

```C
#define DELAY(count) {for (int time = 0; time < count; time++);}

/*
 *  Не уверен точно, но похоже аналовый сигнал может выходить только с ноги PA5 и PA4
 * */

int main(void) {
    //// GPIO init
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER |= ((GPIO_MODER_MODER5_1 | GPIO_MODER_MODER5_0) |
                     (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER4_0));

    //// DAC init
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR |= ((DAC_CR_EN1) | (DAC_CR_EN2));

    while (1) {
        GPIOD->ODR ^= GPIO_ODR_OD12;
        for (uint16_t i = 0, j = 4096; (i < 4096) && (j > 0); i++, j--) {
            DAC->DHR12R1 = i;
            DAC->DHR12R2 = j;
            DELAY(1600);
        }
    }
}
```

**Источники:**
* [Переведенная документаци](http://microsin.net/programming/arm/stm32f4xx-digital-to-analog-converter.html)
* [github1](https://github.com/goktugh/Arduino-Stm32f407-Dac/blob/master/src/Dac_STM32.cpp)
* [github2](https://github.com/dogukan-bicer/stm32f407_discovery_dac_registers/blob/main/main.c)
* [github3](https://github.com/Dogukan1412/STM32F407-DAC-EXAMPLE-REGISTER-CODING/blob/main/DAC_Register_Coding/src/main.c)


## I2C

Шина I2C (англ. *Inter-Integrated Circuit*  - в переводе *Межинтегральная схема*) - последовательная асимметричная шина для связи между интегральными схемами внутри электронных приборов. Использует две двунаправленные линии связи (SDA и SCL), применяется для соединения низкоскоростных периферийных компонентов с процессорами и микроконтроллерами (например, на материнских платах, во встраиваемых системах, в мобильных телефонах).

изначально была разработана компанией Philips Semiconductors (теперь NXP) в 1982 году. Созданно 6 поколений.

###### Принцип подключения и работы
Шина I2C синхронная, состоит из двух линий: данных (SDA) и тактирования (SCL). Есть ведущий (master) и ведомые (slave). Инициатором обмена всегда выступает ведущий, обмен между двумя ведомыми невозможен. Всего на одной двухпроводной шине может быть до 127 устройств.

![](./img/stm32/I2C_master_slave_1.png)
![](./img/stm32/I2C_master_slave_2.png)

Такты на линии SCL генерирует master. Линией SDA могут управлять как мастер, так и ведомый в зависимости от направления передачи. Единицей обмена информации является пакет, обрамленный уникальными условиями на шине, именуемыми стартовым и стоповым условиями. Мастер в начале каждого пакета передает один байт, где указывает адрес ведомого и направление передачи последующих данных. Данные передаются 8-битными словами. После каждого слова передается один бит подтверждения приема приемной стороной.

![](./img/stm32/I2C_data_transfer.png)

I²C использует две двунаправленные линии, подтянутые к напряжению питания и управляемые через открытый коллектор или открытый сток — последовательная линия данных (SDA, [англ.](https://ru.wikipedia.org/wiki/%D0%90%D0%BD%D0%B3%D0%BB%D0%B8%D0%B9%D1%81%D0%BA%D0%B8%D0%B9_%D1%8F%D0%B7%D1%8B%D0%BA "Английский язык") Serial DAta) и последовательная линия [тактирования](https://ru.wikipedia.org/wiki/%D0%A2%D0%B0%D0%BA%D1%82%D0%B8%D1%80%D0%BE%D0%B2%D0%B0%D0%BD%D0%B8%D0%B5 "Тактирование") (SCL, [англ.](https://ru.wikipedia.org/wiki/%D0%90%D0%BD%D0%B3%D0%BB%D0%B8%D0%B9%D1%81%D0%BA%D0%B8%D0%B9_%D1%8F%D0%B7%D1%8B%D0%BA "Английский язык") Serial CLock). Стандартные напряжения +5 В или +3,3 В.

**STM32 I2C в режиме Slave**. По умолчанию аппаратура I2C находится в режиме slave. Для переключения в режим master необходима генерация сигнала START. Для обеспечения корректных таймингов шины тактовая частота периферии должна быть запрограммирована в регистр I2C_CR2. Исходная входная частота периферии должна быть как минимум:

• 2 МГц для режима скорости Sm (режим стандартной скорости 100 кГц)  
• 4 МГц для режима скорости Fm (режим повышенной скорости 400 кГц)

При обнаружении сигнала START от внешнего master принимаемый адрес с сигнала SDA последовательно поступает в регистр сдвига. Затем он сравнивается с запрограммированными (в регистрах (I2C_OAR1 и (I2C_OAR2) адресами. После получения адреса интерфейс slave-устройства с помощью регистра сдвига принимает байты через сигнал SDA в регистр I2C_DR. После каждого байта интерфейс генерирует импульс подтверждения, если установлен бит ACK.

Если установлен флаг RxNE, и данные не были прочитаны из регистра I2C_DR до окончания приема следующего байта, установится бит BTF и интерфейс будет ожидать, пока не будет очищен бит BTF путем чтения регистра I2C_SR1, за которым идет чтение регистра I2C_DR. Пока не будет очищен бит BTF, интерфейс slave будет растягивать сигнал SCL (подтягивая его к уровню LOW), давая тем самым устройству master сигнал неготовности к получению дальнейших данных. Дальнейшая передача данных будет невозможна, пока slave не освободит линию сигнала SCL. По этой причине следует быть осторожным с функцией clock stretching в устройствах slave.

После того, как передан последний байт данных, master генерирует сигнал STOP. Интерфейс определит этот сигнал, установит бит STOPF и сгенерирует прерывание, если установлен бит ITEVFEN. Бит STOPF очищается путем чтения регистра SR1, за которым идет запись регистра CR1.

**STM32 I2C в режиме Master**. В этом режиме интерфейс инициирует передачу данных и генерирует сигнал тактов SCL. Передача данных всегда начинается с сигнала START, и заканчивается сигналом STOP. Режим Master выбирается сразу, как только на шине генерируется сигнал START. Дальнейшая генерация последовательности необходимых базовых элементов протокола I2C происходит в режиме master.

Типовая последовательность действий программы для организации работы в режиме master:

• Программируется входная тактовая частота периферийного устройства I2C в регистре I2C_CR2, чтобы обеспечить корректные тайминги шины I2C.  
• Конфигурируются регистры управления тактами.  
• Конфигурируется регистр времени нарастания уровня.  
• Программируется регистр I2C_CR1, чтобы разрешить работу периферийного устройства I2C.  
• Устанавливается бит START в регистре I2C_CR1, чтобы сгенерировать сигнал START.

Исходная входная частота периферии должна быть как минимум (требования такие же, как и для режима slave):

• 2 МГц для режима скорости Sm (режим стандартной скорости 100 кГц)  
• 4 МГц для режима скорости Fm (режим повышенной скорости 400 кГц)

Схема в stm32f4(скорее всего в остальных такая же):

![](./img/stm32/I2C_block_diagram.png)
###### Регистры

![](./img/stm32/I2C_CR1.png)
* **SWRST**: Сброс программного обеспечения
	Когда он установлен, I2C находится в состоянии сброса. Перед сбросом этого бита убедитесь, что линии I2C разъединены и шина свободна.
		0: Периферийное устройство I2C не находится в режиме сброса
		1: Периферийное устройство I2C находится в состоянии сброса
	*Примечание: Этот бит можно использовать для повторной инициализации периферийного устройства после ошибки или заблокированного состояния. В качестве примера, если бит ЗАНЯТОСТИ установлен и остается заблокированным из-за сбоя на шине, бит SWRST может быть использован для выхода из этого состояния.*

* **ALERT**: SMBus предупреждение
	Этот бит устанавливается и очищается программным обеспечением и очищается аппаратным обеспечением, когда PE=0.
		0: Освобождает высокий вывод SMBA. Заголовок адреса ответа на предупреждение, за которым следует NACK. 
		1: Вывод SMBA находится на низком уровне. Заголовок адреса ответа на предупреждение, за которым следует подтверждение.

* **PEC**: Проверка ошибок пакетов
	Этот бит устанавливается и очищается программным обеспечением и очищается аппаратным обеспечением при передаче PEC, или при условии запуска или остановки, или когда PE=0.
		0: Передача PEC невозможна
		1: Передача PEC (в режиме Tx или Rx)
	*Примечание: Расчет PEC искажен из-за арбитражного проигрыша.*

* **POS**: Подтверждение/PEC позиция(для получения данных)
	Этот бит устанавливается и очищается программным обеспечением и аппаратно, когда PE=0.
		0: Бит подтверждения управляет (N) подтверждением текущего байта, принимаемого в регистр сдвига. Бит PEC указывает на то, что текущий байт в регистре сдвига является PEC.
		1: Бит подтверждения управляет (N) подтверждением следующего байта, который будет получен в регистре сдвига. Бит PEC указывает на то, что следующий байт в регистре сдвига является PEC
	*Примечание: Бит POS должен использоваться только в 2-байтовой конфигурации приема в главном режиме. Он должен быть сконфигурирован перед началом приема данных, как описано в процедуре приема 2 байт, рекомендованной в разделе: Главный приемник на стр. 849.*

* **ACK**: Подтвердить включение
	Этот бит устанавливается и очищается программным обеспечением и аппаратно, когда PE=0. 
		0: Подтверждение не возвращено
		1: Подтверждение возвращается после получения байта (совпадающий адрес или данные)

* **STOP**: Остановить генерацию
	Бит устанавливается и сбрасывается программным обеспечением, сбрасывается аппаратным обеспечением при обнаружении состояния остановки, устанавливается аппаратным обеспечением при обнаружении ошибки тайм-аута.
		В главном режиме:
			0: Генерация останова отсутствует.
			1: Остановите генерацию после передачи текущего байта или после отправки текущего условия запуска. 
		В подчиненном режиме:
			0: Генерация останова отсутствует.
			1: Освободите строки SCL и SDA после передачи текущего байта.

* **START**: Начать генерацию
	Этот бит устанавливается и очищается программным обеспечением и очищается аппаратным обеспечением при отправке start или PE=0.
		В главном режиме:
			0: Нет начальной генерации
			1: Повторная генерация запуска
		В подчиненном режиме:
			0: Нет начальной генерации
			1: Запустите генерацию, когда шина освободится

* **NOSTRETCH**: Отключение растягивания часов (подчиненный режим)
	Этот бит используется для отключения растягивания тактовой частоты в подчиненном режиме, когда установлен флаг ADDR или BTF, до тех пор, пока он не будет сброшен программным обеспечением.
		0: Включено растягивание часов
		1: Отключено растягивание часов

* **ENGC**: Включение общего вызова
	0: Общий вызов отключен. Адрес 00h голый. 
	1: Общий вызов включен. Адрес 00h сохранен.

* **ENPEC**: Включение PEC
	0: Расчет PEC отключен
	1: Включен расчет PEC

* **ENARP**: Включить ARP
	0: ARP отключен
	1: Включение ARP
	Адрес устройства SMBus по умолчанию распознан, если CMBTYPE=0 Адрес узла SMBus распознан, если CMBTYPE=1

* **SMBTYPE**: Тип шины SM
	0: Устройство SMBus 
	1: Узел SMBus

* **SMBUS**: SMBusmode 
	0: I2C mode
	1: SMBus mode

* **PE**: Включение периферийного устройства 
		0: Отключение периферийного устройства
		1: Включение периферийного устройства
	*Примечание: Если этот бит сбрасывается во время продолжения связи, периферийное устройство отключается по окончании текущей связи, когда возвращается в состояние ожидания.
	Все сбросы битов из-за PE=0 происходят в конце передачи данных.
	В главном режиме этот бит не должен быть сброшен до окончания обмена данными.

###### Код

```C
void I2C1_init() {
    //// Настройка GPIO
    // PB6 -> SCL, BP7 ->SDA
    // 0. Включаем тактирование
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 1. Настраиваем скорость(Максимальную)
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR6;
    
    // 2. Настраиваем на альтернативную функцию
    GPIOB->MODER |= GPIO_MODER_MODER7_1 | GPIO_MODER_MODER6_1;
    
    // 3. Установите выход на открытый слив 
    GPIOB->OTYPER |= GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_6;
    
    // 4. Настраиваем на релим PULL-UP
    GPIOB->PUPDR |= GPIO_PUPDR_PUPDR7_0 | GPIO_PUPDR_PUPDR6_0;
    
    // 5. Включаем альтернативную функцию
    GPIOB->AFR[0] |= GPIO_AFRL_AFRL7_2 | GPIO_AFRL_AFRL6_2;
    
    //// Настройка I2C
    // 0. Включаем тактирование
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN ;
    
    // 1. Устанавливает тактовую частоту периферийного устройства
    I2C1->CR2 = 0x2A;
    
    // 2. Задаем частоту работы, есть два режима стандартная и повышенная, выставлена стандартная
    I2C1->CCR = 0xD2;
    
    // 3. Настройка скорости
    I2C1->TRISE = 0x2B;
    
    // 4. Включаем переферию
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2C1_write_bytes(uint8_t addr, uint8_t *data, uint8_t len) {
	// 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);
	
    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
	
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));
	
    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (addr << 1);

	// 4. Ждем пока предет подверждение получения адреса
	while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. Считываем SR1 и SR2 для того чтобы сбросить их
	(void) I2C1->SR2;

	// 6. Начинаем запись массива
	for (int i = 0; i < len; i++, *data++){
		I2C1->DR = *data;
		
        // 6.1. Ждем потверждения передачи
		while (!(I2C1->SR1 & I2C_SR1_BTF));
	}

	// 7. Выключаем передачу устанавливая стоповый бит
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_write_code(uint8_t address, uint8_t code, uint8_t data) {
    // 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);

    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
    
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));

    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (address << 1);

	// 4. Ждем пока предет подверждение получения адреса	
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. Считываем SR1 и SR2 для того чтобы сбросить их
	(void) I2C1->SR2;

	// 4. Передаем код
	I2C1->DR = code;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 7. Передаем данные
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 8. Останавливаем передачу
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C_write_byte(uint8_t address, uint8_t data) {
    // 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);

    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
    
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));

    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (address << 1);

	// 4. Ждем пока предет подверждение получения адреса	
    while (!(I2C1->SR1 & I2C_SR1_ADDR));

	// 5. Считываем SR1 и SR2 для того чтобы сбросить их
	(void) I2C1->SR2;

	// 4. Передаем код
	I2C1->DR = data;
	while (!(I2C1->SR1 & I2C_SR1_TXE));

	// 5. Останавливаем передачу
	I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_read_bytes(uint8_t address, uint8_t *data, uint8_t size) {
    while (I2C1->SR2 & I2C_SR2_BUSY);
    
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    // Повторяющиеся шаги я расписывать не буду
    // 1. Включаем подтверждение возвращается после получения байта
    I2C1->CR1 |= I2C_CR1_ACK;
    
    // 2. Отправляем в канал адрес, для того чтобы происходила чтение данных \
    //    его надо сместить в лево на 1 бит и установим 1 первым битом
    I2C1->DR = ((address << 1) | 0x01);
    while (!(I2C1->SR1 & I2C_SR1_ADDR))
    (void) I2C1->SR2;
    
    // 3. Считывание массива данных
    while (size--) {
        // 3.1. Ждем, что регистр данных не пуст
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        *data++ = I2C1->DR;
    }

    I2C1->CR1 |= I2C_CR1_STOP;

    // 4. Выключаем подтверждение возвращается после получения байта 
    I2C1->CR1 &=~ I2C_CR1_ACK;
}

/** @breif I2C_read_byte - при работе с некоторой перефирией продуман сценарий \
 *                         получения 1 байта передачей команды
 * */
uint8_t I2C_read_byte(uint8_t address, uint8_t code) {
    while (I2C1->SR2 & I2C_SR2_BUSY);

	uint8_t data; // Возращаемые данные 
	//// В начале передадим код регистра, который надо считать, а после считаем регистр
    // 0. Передача кода
    I2C_write_byte(address, code);
	
    // 1. Чтение данных 
	I2C1->CR1 |= I2C_CR1_START;
	while (!(I2C1->SR1 & I2C_SR1_SB));

    I2C1->CR1 |= I2C_CR1_ACK;

	I2C1->DR = ((address << 1) | 0x1);
	while (!(I2C1->SR1 & I2C_SR1_ADDR));
	(void) I2C1->SR2;

	while (!(I2C1->SR1 & I2C_SR1_RXNE));
	data = I2C1->DR;

	I2C1->CR1 |= I2C_CR1_STOP;
    I2C1->CR1 &= ~I2C_CR1_ACK;
	return data;
}
```

**Источники:**
* [Частично переведенная документация](http://microsin.net/programming/arm/stm32-hal-i2c-functionsexamples-slave-master-interrupt.html)
* [Парень с кибер-форума](https://www.cyberforum.ru/arm/thread2867781.html)
* [MCLEANS](https://github.com/MCLEANS/STM32F4_I2C_CONFIGURATION/blob/master/LIBRARY/I2C.cpp)
* [вики про I2C](https://ru.wikipedia.org/wiki/I²C)
* [Ещё один человек с github](https://github.com/RafPe/STM32F4DISCOVERY-I2C/blob/master/RafPe_STM32F407_I2C.c)
* [Примеры с русскими комментариями](https://github.com/TakitaNirasa/stm32f405_i2c/blob/main/stm32f4xx_i2c.c)
* [Статья про I2C на каком-то сайте](https://hubstub.ru/stm32/184-stm32-i2c.html)
* [На гитхабе остановился на этой странице](https://github.com/search?q=stm32f4_I2C&type=repositories&p=4)

## SPI
**SPI** (*Serial Peripheral Interface*, *SPI bus* — последовательный периферийный интерфейс, шина SPI) — последовательный синхронный стандарт передачи данных в режиме полного дуплекса, предназначенный для обеспечения простого и недорогого высокоскоростного сопряжения микроконтроллеров и периферии. SPI также иногда называют четырёхпроводным (four-wire) интерфейсом. 

![](./img/stm32/SPI_single_slave.png)

Этот интерфейс можно сконфигурировать либо как в режиме главного устройства (master), либо как в режиме подчиненного устройства (slave). В режиме главного устройства SPI формирует выходной сигнал тактов (SCK) для внешнего подчиненного устройства. Интерфейс позволяет также работать в конфигурации с несколькими главными устройствами на шине (конфигурация multimaster).

Схема в stm32f4:

![](./img/stm32/SPI_block_diagram.png)

Обычно SPI подключается к внешним устройствам через 4 сигнала, 3 сигнала настраиваются как альтернативные функции:
- MOSI — выход ведущего, вход ведомого (от англ. Master Out Slave In). Служит для передачи данных от ведущего устройства ведомому.
- MISO — вход ведущего, выход ведомого (от англ. Master In Slave Out). Служит для передачи данных от ведомого устройства ведущему.
- SCLK или SCK — последовательный тактовый сигнал ( от англ. Serial Clock). Служит для передачи тактового сигнала для ведомых устройств.
- CS или SS или NSS — выбор микросхемы, выбор ведомого ( от англ. Chip Select, Slave Select). *NSS: Node Slave Select*. Этот сигнал необязателен для использования, применяется как аппаратная выборка slave-устройства. Сигнал позволяет главному устройству шины SPI выбрать определенное подчиненное устройство, чтобы избежать конфликта на сигналах данных. Сигналы выборки NSS на главном устройстве могут также вырабатываться программно, управлением ножками GPIO. Ножка NSS также может использоваться как выход, если это разрешено (битом SSOE), и переключаться в лог. 0 при обращении к внешним подчиненным устройствам, если SPI работает в конфигурации master. Таким образом, все ножки NSS устройств, подключенных к сигналу Master NSS, видят лог. 0 и становятся подчиненными по низкому уровню, когда они сконфигурированы в аппаратном режиме выборки NSS. Когда конфигурируется режим master, и NSS конфигурируется как вход (MSTR=1 и SSOE=0), и если NSS переводится в лог. 0, то SPI входит в состояние отказа главного устройства (master mode fault state): бит MSTR автоматически очищается, и устройство конфигурируется в режиме подчиненного устройства, slave mode (см. далее раздел "Флаги ошибок").
Пример соединения Master и Slave:

![](./img/stm32/SPI_master_slave.png)

Все одноименные выводы соединяются друг с другом. Таким образом, данные последовательно передаются между master и slave, одновременно в двух направлениях (по умолчанию старшим битом MSB вперед).

Обмен всегда инициируется со стороны master-устройства. Когда master передает данные slave-устройству через вывод MOSI, slave-устройство отвечает данными через вывод MISO. Это подразумевает полнодуплексный обмен, засинхронизированный одним тактовым сигналом SCK (этот сигнал генерирует master).

Передача осуществляется пакетами. Длина пакета, как правило, составляет 1 байт (8 бит), при этом известны реализации SPI с иной длиной пакета, например, 4 бита. Ведущее устройство инициирует цикл связи установкой низкого уровня на выводе выбора подчиненного устройства (SS) того устройства, с которым необходимо установить соединение. При низком уровне сигнала SS:

![](./img/stm32/SPI_8bit.png)

Подлежащие передаче данные ведущее и ведомое устройства помещают в сдвиговые регистры. После этого ведущее устройство начинает генерировать импульсы синхронизации на линии SCLK, что приводит к взаимному обмену данными. Передача данных осуществляется бит за битом от ведущего по линии MOSI и от ведомого по линии MISO. Передача осуществляется, как правило, начиная со старших битов, но некоторые производители допускают изменение порядка передачи битов программными методами. После передачи каждого пакета данных ведущее устройство, в целях синхронизации ведомого устройства, может перевести линию SS в высокое состояние.

**Фаза (* Clock PHAse, фаза тактов*) и полярность (*Clock POLarity, полярность тактов*) тактов**. Программно можно выбрать 4 варианта взаимодействия по шине SPI, используя биты CPOL и CPHA в регистре SPI_CR1. Бит CPOL (Clock POLarity, полярность тактов) управляет устойчивым (т. е. исходным) состоянием сигнала тактов, когда данные не передаются. Этот бит влияет на оба режима, master и slave. Если CPOL=0, то ножка SCK в режиме ожидания находится в лог. 0. Если CPOL=1, то ножка SCK в режиме ожидания находится в лог. 1. Если установлен бит CPHA (Clock PHAse, фаза тактов), то второй перепад на выводе SCK (спад в случае CPOL=0, фронт в случае CPOL=1) служит стробом бита данных (по умолчанию MSB). Данные защелкиваются при появлении второго перепада тактов. Если CPHA=0, то стробом служит первый перепад на выводе SCK (спад уровня при CPOL=1, нарастание уровня при CPOL=0). Данные защелкиваются при появлении первого перепада тактов.

Комбинация состояния бит CPOL и CPHA выбирает тактовый перепад для захвата данных.

![](./img/stm32/SPI_timing_diagram.png)

- CPOL = 0 — исходное состояние сигнала синхронизации -- низкий уровень;
- CPOL = 1 — исходное состояние сигнала синхронизации -- высокий уровень;
- CPHA = 0 — выборка данных производится по переднему фронту (переключению) сигнала синхронизации. То есть по переключению из основного в противоположное ему;
- CPHA = 1 — выборка данных производится по заднему фронту (переключению) сигнала синхронизации. То есть по переключению обратно к основному из противоположного;

Для обозначения режимов работы интерфейса SPI принято следующее соглашение:

- режим 0 (CPOL = 0, CPHA = 0);
- режим 1 (CPOL = 0, CPHA = 1);
- режим 2 (CPOL = 1, CPHA = 0);
- режим 3 (CPOL = 1, CPHA = 1).

**Как выглядит подключение нескольких модулей на SPI:**

![](./img/stm32/SPI_many_sensor.png)

###### Код
```C
void GPIO_init() {
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	GPIOE->MODER |= GPIO_MODER_MODER3_0;
	GPIOE->ODR |= GPIO_ODR_OD3;
}

#define SCK_PIN 5
#define MOSI_PIN 7
#define MISO_PIN 6

#define SPI_PRESCALER 64
#define CPHA 0
#define CPOL 0
#define LSBFIRST 1

void SPI1_init() {
	//// GPIO
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1);
	GPIOA->AFR[0] |= ((GPIO_AFRL_AFSEL5_2 | GPIO_AFRL_AFSEL5_0) |
					  (GPIO_AFRL_AFSEL6_2 | GPIO_AFRL_AFSEL6_0) |
					  (GPIO_AFRL_AFSEL7_2 | GPIO_AFRL_AFSEL7_0));
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_0 | GPIO_OSPEEDER_OSPEEDR6_0 | GPIO_OSPEEDER_OSPEEDR7_0);
	GPIOA->PUPDR |= (GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1);

	//// SPI
	// 0. Включаем тактирование
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	// 1. Настраиваем на Master Mode
	SPI1->CR1 |= SPI_CR1_MSTR;
	
	// 2. Отключаем
	// однонаправленный режим работы по одной линии данных
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	// выход запрещен (режим "только прием"), теперь работает одновременные передача и прием
	SPI1->CR1 &= ~SPI_CR1_RXONLY;
	
	// 3. Настраиваем прием на 8 бит
	SPI1->CR1 &= ~SPI_CR1_DFF;
	
	// 4. Настройка управления slave устройством
	SPI1->CR1 |= (SPI_CR1_SSI | SPI_CR1_SSM);
	
	// 5. Управление скоростью передачи 000: fPCLK/2
	SPI1->CR1 &= ~SPI_CR1_BR;
	
	// 6. Установка передачи в первый режим MSB
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
	
	// 7. Настройка полярности тактов(CPOL) и фазы тактов(CPHA)
	SPI1->CR1 &= ~SPI_CR1_CPHA;
	SPI1->CR1 &= ~SPI_CR1_CPOL;
	
	// 8. Включить CRC
	SPI1->CR1 |= SPI_CR1_CRCEN;
	
	// 9. Выбор формата Motorola, какой то внутренний, пока не понятно что это
	SPI1->CR2 = 0x0000;
	
	// 10. Включаем SPI
	SPI1->CR1 |= SPI_CR1_SPE;

// Ещё 1 пример реализации SPI
#if 0
	SPI1->CR1 &= ~SPI_CR1_SPE;
	
	if(SPI_PRESCALER == 2) SPI1->CR1 &= ~(SPI_CR1_BR);
	if(SPI_PRESCALER == 4) SPI1->CR1 |= SPI_CR1_BR_0;
	if(SPI_PRESCALER == 8) SPI1->CR1 |= SPI_CR1_BR_1;
	if(SPI_PRESCALER == 16) SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1;
	if(SPI_PRESCALER == 32) SPI1->CR1 |= SPI_CR1_BR_2;
	if(SPI_PRESCALER == 64) SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_0;
	if(SPI_PRESCALER == 128) SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1;
	if(SPI_PRESCALER == 256) SPI1->CR1 |= SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0;

	SPI1->CR1 |= SPI_CR1_MSTR;

	SPI1->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM;

	if(CPHA){
		SPI1->CR1 |= SPI_CR1_CPHA;
	} else {
		SPI1->CR1 &= ~SPI_CR1_CPHA;
	}

	if(CPOL){
		SPI1->CR1 |= SPI_CR1_CPOL;
	} else {
		SPI1->CR1 &= ~SPI_CR1_CPOL;
	}

	if(LSBFIRST){
		SPI1->CR1 |= SPI_CR1_LSBFIRST;
	} else {
		SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
	}

	SPI1->CR2 |= SPI_CR2_SSOE;

	SPI1->CR1 |= SPI_CR1_SPE;
#endif
}

uint16_t SPI1_write(uint8_t data) {
	uint16_t buffer = 0;
	while(!((SPI1->SR) & SPI_SR_TXE)){}
	SPI1->DR = data;
	while(!(SPI1->SR & SPI_SR_RXNE)){}
	buffer = SPI1->DR; // После того как произвели запись надо считать значение из SPIx->DR
	return buffer;
}

uint16_t SPI1_read(uint8_t address) {
	uint16_t data = 0;
	data = SPI1_write(address);
	data = SPI1_write(0x00);
	return data;
}
```

#### Код 2

```C
#define ch3	(0x03 << 25)

/*
 * @brief initialize SPI1 peripheral and required pins
 * @param  None
 * @retval None
 */
void spi1_init(void) {
	//enable clock for GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//set PA5, PA6 and PA7 to alternate function mode
	GPIOA->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1);
	//set which type of alternate function is
	GPIOA->AFR[0] |= ((0x05<<20) | (0x05<<24) | (0x05<<28));
	//enable clock access to SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	//set software slave managment
	SPI1->CR2 |= SPI_CR2_TXDMAEN;
	SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_SPE);
	//set SPI in master mode
//	SPI1->CR1 |= SPI_CR1_MSTR;
//	SPI1->CR1 |= SPI_CR1_BR_0;
	//enable DMA_TX buffer
//	SPI1->CR2 |= SPI_CR2_TXDMAEN;
	//enable SPI peripheral
//	SPI1->CR1 |= SPI_CR1_SPE;
}

/*
 * @brief intialize DMA2 Stream3 Channel 3
 * @param  None
 * @retval None
 */
void dma2_stream3_ch2_init(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	DMA2_Stream3->CR &= ~DMA_SxCR_EN;
	while ((DMA2_Stream3->CR) & DMA_SxCR_EN){;}
	DMA2_Stream3->CR = (ch3 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE 
                            | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_CIRC);
	//DMA2_Stream3->FCR=0;
	DMA2_Stream3->FCR &= ~DMA_SxFCR_DMDIS;
	NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

/*
 * @brief initiation DMA transfer
 * @param  uint32_t src for data source
 * @param  uint32_t length of data source
 * @retval None
 */
void spi_transfer_dma(uint32_t src,uint32_t len) {
	DMA2->LIFCR |= (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
	DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;
	DMA2_Stream3->M0AR = src;
	DMA2_Stream3->NDTR = len;
	DMA2_Stream3->CR |= DMA_SxCR_EN;
}

#define	LED_red		(1U << 14)
#define LED_green  	(1U << 12)
#define LED_blue    (1U << 15)
#define LED_orange  (1U << 13)
#define	LED_PORT    GPIOD
void led_init();

#define GPIOD_CLOCK    (1 << 3)
//bits of the pins
#define LED_red_BIT    (1U << 28)
#define LED_green_BIT  (1U << 24)
#define LED_orange_BIT (1U << 26)
#define LED_blue_BIT   (1U << 30)
//LED port
#define	LED_PORT		GPIOD
//bit possition for color
#define	LED_red		   (1U << 14)
#define LED_green  	   (1U << 12)
#define LED_blue       (1U << 15)
#define LED_orange     (1U << 13)

/*
 * @brief initialize the built-in LED of STM32F407-DISCO
 * @param  None
 * @retval None
 */
void led_init() {
	RCC->AHB1ENR |= GPIOD_CLOCK;
	GPIOD->MODER |= (LED_red_BIT | LED_green_BIT | LED_orange_BIT | LED_blue_BIT);
}

char data[] = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};
volatile uint8_t finished = 0;

void delayMs(int delay) {
    volatile int i;
    for (; delay > 0; delay--) {
        for (i = 0; i < 3195; i++);
    }
}

int main(void) {
	//printf("system init\r\n");
	led_init();
	spi1_init();
	dma2_stream3_ch2_init();
	//printf("system init finished\r\n");
    uint8_t len = 7;

    DMA2->LIFCR |= (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
	DMA2_Stream3->PAR = (uint32_t)&SPI1->DR;
	DMA2_Stream3->M0AR = (uint32_t)&data[0];
	DMA2_Stream3->NDTR = len;
	DMA2_Stream3->CR |= DMA_SxCR_EN;


	while(1) {
//		spi_transfer_dma((uint32_t)data,sizeof(data));
//		while(finished==0){}
//		finished=0;
//		delayMs(10);
	}
}

void DMA2_Stream3_IRQHandler(void) {
    if (DMA2->LISR & (DMA_LISR_TCIF3)) {
		LED_PORT->ODR ^= LED_blue;
		//printf("finished transfered\r\n");
		finished = 1;
		//DMA2_Stream3->CR &= ~DMA_SxCR_EN;
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
	}

	if (DMA2->LISR & (DMA_LISR_HTIF3)) {
		LED_PORT->ODR ^= LED_green;
		//printf("half transfered\r\n");
		DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
	}


    if (DMA2->LISR & (DMA_LISR_TEIF3)) {
		//printf("transfer error interrupt\r\n");
		DMA2->LIFCR |= (DMA_LIFCR_CTEIF3);
	}

    if (DMA2->LISR & (DMA_LISR_DMEIF3)) {
		//printf("Direct mode interrupt error\r\n");
		DMA2->LIFCR |= (DMA_LIFCR_CDMEIF3);
	}
    
    if (DMA2->LISR & (DMA_LISR_FEIF3)) {
		//printf("FIFO error interrupt\r\n");
		DMA2->LIFCR |= (DMA_LIFCR_CFEIF3);
	}

//	NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);
}

```
Говоря о коде, надо быть внимательнее для каждого датчика, так как некоторые похоже очень странно работают, не знаю почему, но код не в **#if** **#else** не работал с датчиком на зеленой плате (**LIS302DL**). 

**Источники:**
* [Wiki, картинки красивые](https://ru.wikipedia.org/wiki/Serial_Peripheral_Interface)
* [Переведенная документация, переведены и регистры, чего нет в I2C](http://microsin.net/programming/arm/stm32f4xx-spi.html)
* [Статья про SPI на хабре](https://habr.com/ru/articles/123145/)
* [Пример работы с датчиком на плате](https://github.com/yusuf7855/Register-level-code-for-interfacing-LIS302DL-accelerometer-with-STM32F407/blob/main/Core/Src/main.c)
* [~~Код на регистрах так и не смотрел~~](https://github.com/hussamaldean/SPI_TX_DMA_STM32F407/blob/main/spi.c)
* [Пример работы другого датчика для голубой платы](https://github.com/hemantnile/stm32f4-discovery-SPI/blob/master/src/main.c)
* [MCLEANS](https://github.com/MCLEANS/STM32F4_SPI_CONFIGURATION/blob/master/IMPLEMENTATION/SPI.cpp)
* [Опять какой то пример с датчиком для голубой платы, но просто структура запомнилась](https://ivariousblog.blogspot.com/2017/07/stm32f407vg-spi.html)
* [~~Ну тут как бы пример, но в тоже время не сказать, чтобы прям помогло~~](https://github.com/jerl1/lisd3dsh/blob/master/lis3dsh.h)
* [Хорошие примеры про работу с датчиком LIS302DL](https://easystm32.ru/sensors/49-lis302dl/)
* [Примеры с хабра, про акселерометры и гироскопы](https://habr.com/ru/articles/366461/)
* [Страница на которой остановился](https://github.com/search?q=stm32f4_SPI&type=repositories&p=5)

## RCC

## RTC

## DCMI

## LTDC

## SDIO

Надо использовать эту переферию для работы с SD картой. 

## Набор красивых макросов

```C
#define SET_BIT(REG, BIT)     ((REG) |= (BIT)) 
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT)) 
#define READ_BIT(REG, BIT)    ((REG) & (BIT)) 
#define CLEAR_REG(REG)        ((REG) = (0x0)) 
#define WRITE_REG(REG, VAL)   ((REG) = (VAL)) 
#define READ_REG(REG)         ((REG)) 
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL))) 
```






