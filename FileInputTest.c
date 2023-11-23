#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main()
{
    int i, j;
    double *elas_mode = (double *)malloc(66655 * 16 * sizeof(double));
    memset(elas_mode, 0, 66655 * 16 * sizeof(double)); // initialize elas_mode

    FILE *fp = fopen("elas_mode.txt", "r");

    for (i = 0; i < 66655; i++) // fill the array
        for (j = 0; j < 16; j++)
            if (fscanf(fp, "%lf", elas_mode + i * 16 + j) == 0 || fscanf(fp, "%lf", elas_mode + i * 16 + j) == EOF)
                break;

    printf("%d, %d\n", i, j);

    fclose(fp);
    free(elas_mode);
    return 0;
}