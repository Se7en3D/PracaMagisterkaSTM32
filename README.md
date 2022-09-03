Diagram UML programu:

![obraz](https://user-images.githubusercontent.com/77826670/188262567-835c7131-756f-4d94-b338-7478f1c75ae7.png)

Ogólny opis działania programu:

![obraz](https://user-images.githubusercontent.com/77826670/188262594-859701b6-668e-47c8-b5a6-2c1600206dde.png)

Maszyna stanów dla czujnika laserowego:

![obraz](https://user-images.githubusercontent.com/77826670/188262610-17f515c0-b24f-4eac-b001-ad8ecf93d667.png)

Opis poszczególnych stanów:
-	vl53l0x_NoInitialized – brak inicjalizacji. Niemożliwe jest wykonanie pomiaru odległości. Moduł automatycznie wykona procedurę inicjalizacji podczas wywołania funkcji stanów;
-	vl53l0x_InitializedError – błąd inicjalizacji układu, wymagana jest reakcja użytkownika w celu usunięcia problemu,
-	vl53l0x_Idle – oczekiwanie na wywołanie funkcji pomiarowej albo automatyczne przejście do pobrania pomiaru, jeśli ustawiony jest tryb pracy ciągłej,
-	vl53l0x_WaitingForTheInterruptFlagToBeSet – oczekiwanie na ustawienie flagi przerwania,
-	vl53l0x_WaitUntilStartBitHasBeenCleared – oczekiwanie na wyzerowanie bitu startu pomiaru,
-	vl53l0x_Timeout – upłynął czas wymagany na wykonanie pomiaru. Wysyłany jest błąd a następnie maszyna stanów przechodzi do Vl53l0x_Idle.

Maszyna stanów dla układu pomiarowego składającego się z czujnika laserowego, ultradziękowego oraz serwomechanizmu:

![obraz](https://user-images.githubusercontent.com/77826670/188262708-310518e6-d1ad-4706-aecf-765ee19f231c.png)

Opis poszczególnych stanów:
-	rangeMeasurmentIdle – oczekiwanie na użycie funkcji rangeMeasurment() rozpoczynającej zmianę pozycji serwomechanizmu oraz pomiar odległości;
-	rangeMeasurmentPostitionChanging – oczekiwanie, aż funkcja isReady() (rozdział 5.3) zwróci wartość 1 informującą o zakończeniu przemieszczenia się serwomechanizmu do zadanej pozycji. Od razu rozpoczynany jest również pomiar od czujnika laserowego i ultradźwiękowego;
-	rangeMeasurmentDistance – oczekiwanie na pomiar odległości od czujnika laserowego oraz ultradźwiękowego. Ze względu, że moduły kończą pomiar w różnych odstępach czasowych, zastosowano dwie zmienne informujące o pobraniu pomiaru dla każdego układu z osobna. Dopiero gdy obie zmienne zostaną ustawione maszyna stanów przechodzi do   rangeMeasurmentEnd. W przypadku, gdy któryś z moduł został rozłączony maszyna przechodzi do następnego stanu po upłynięciu konkretnej wartości czasu;
-	rangeMeasurmentEnd – oczekiwanie na odebranie pomiarów od dwóch czujników odległości. W stanie tym można wykonać kolejny pomiar bez konieczności pobrania wyników.
