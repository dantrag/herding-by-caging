#include <stdio.h>
#include <iostream>

int main()
{
  int count = 0;
  freopen("new.dae", "w", stdout);
  float d;
  do
  {
    scanf("%f", &d);
    if (count == 1)
    {
      d = 0;
    }
    printf("%.7f ", d);

    count++;
    count %= 3;
  }
  while (d != 666);
  fclose(stdout);
  return 0;
}
