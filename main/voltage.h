#pragma once

typedef enum 
{
    VOLTAGE_OFF,
    VOLTAGE_13V,
    VOLTAGE_18V
} voltage_t;

void voltage_init();
void voltage_select(voltage_t voltage);

// Function to register voltage commands
void register_voltage_commands(void);

