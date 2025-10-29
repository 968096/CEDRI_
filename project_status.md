Change library API - Lorawan - so we can get fragmentation in the headers.
current -> sylvainmontagny/LoRaWAN-Seeed-Grove-Wio-E5 -----



Gas resistance -
temperatura --
sensor id -- nao remover

location id -- remover
Step - talvez tirar
gas valid e heat stable = talvez mudar de bool para int e verificar



1 senssorgpsreading (sensores gerais e infos gerais)
1 vez a cada vez que é acordado


2 sensorvocreading  precisa ser enviado em todo ciclo de leitura ao ser acordado
(bme688)





-------------

forced mode = run, 10 measurements steps, send 5x for 8 sensor, and after that go to sleep 30 minutes.

------------------


fazer alteração no codigo para colocar um duration especifico para cada heater profile  (que seja menos de 30 segundos)-----


check to duty cycle in the firmware, to see if the duration is by heater profile or can be modify or by all

also to check if the heater profile is for all of the sensors or if its idenpedent, ? different duration by different sensors? check if is possible in paralelel mode ------  Forced mode VVVVVV- ---- check if its possible to use to pareallel mode possible to put to sleep or only on forced mode. Use to paralele mode and SLEEP after 10 readings!!! -----