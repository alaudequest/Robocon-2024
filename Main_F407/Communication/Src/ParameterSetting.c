/*
 * ParameterSetting.c
 *
 *  Created on: Mar 18, 2024
 *      Author: SpiritBoi
 */

#include "ParameterSetting.h"
#include "stdbool.h"
NavigationCommandLineInterface nav;
UART_HandleTypeDef *consolePort;
pConsolePrint consolePrint;
bool checkNull = false;
char buffer[100] = {0};
void AddChoice(char *choice);
void CheckNull() {
	if(checkNull) return;
	if(consolePort == NULL || consolePrint == NULL) while (1);
	checkNull = true;
}
void navcli_Init(UART_HandleTypeDef *huart, void (*pConsolePrint)(char*))
{
	consolePrint = pConsolePrint;
	consolePort = huart;
	consolePrint("Type \"cli\" to enter command line interface mode\n");
}

void navcli_ReceiveHandle(UART_HandleTypeDef *huart) {
	if(huart != consolePort) return;
	CheckNull();

}

void Page_Axis() {

}

void Page_Stage() {

}

void Page_ControlType() {

}

void Page_ManualSet() {

}

void Page_TrajectoryPlanning() {

}

void AddChoice(char *choice) {
	strcat(choice, "\n");
}

void navcli_ShowPage() {
	CheckNull();
	consolePrint(buffer);
}
