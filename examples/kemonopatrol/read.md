# How to build "Kemono patrol"

**Kemono patrol** is a simple shooting game on Arduboy using charactor of "Kemono Friends" in Japanese Animation.

Playing movie : [here](https://youtu.be/zvykGUueuCU)

## source
You could download [here](https://twitter.com/metalidol/status/843874920052346881).

and unzip files, you could build on Arduino IDE.

## Plz fix...
### stage.cpp
```
void GameOver::update() {
	if (120 > counter) {
    /*[-]*/ /* drawBitmap(48, 232, serval_1, 36, 64, 1); */
		/*[+]*/ drawBitmap(48, 0, serval_1+36*3, 36, 64-24, 1);
```

```
void Ground::update() {
	/*[-]*//* x--; */
	/*[-]*//* if (x > 160) { x = 159; } */
  /*[+]*/if (x > 0){x--;}else{x = 159;}
}

void Ground::disp() {
  drawBitmap(x - 32, 56, tile, 32, 8, 1);
  /*[+]*/if( x < 32 ){
  /*[+]*/  drawBitmap(0, 56, tile+31-x, x, 8, 1);  
  /*[+]*/}
}
```

Because "drawBitmap" function has been updated.

