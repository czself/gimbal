#include "main.h"
#include "bsp_vofa.h"

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_TX_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_SCE_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void USART6_IRQHandler(void)
{
    uint32_t sr = USART6->SR;

    if (sr & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(USART6->DR & 0xFF);
        vofa_rx_byte(byte);
    }

    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        volatile uint32_t tmp = USART6->DR;
        (void)tmp;
        vofa_rx_err_debug++;
    }
}