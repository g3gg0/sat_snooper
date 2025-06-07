#pragma once

typedef enum 
{
    VOLTAGE_OFF,
    VOLTAGE_13V,
    VOLTAGE_18V
} voltage_t;

void voltage_init();
void voltage_select(voltage_t voltage);
voltage_t voltage_get_current(void); // New function to get current voltage

// Function to register voltage commands
void register_voltage_commands(void);

