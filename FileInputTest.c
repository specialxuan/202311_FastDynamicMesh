#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int read_my_mode(FILE *fp, double *elas_mode)
{
    int i, j;
    for (i = 0; i < 66654; i++) // fill the array
        for (j = 0; j < 16; j++)
            if (fscanf(fp, "%lf", elas_mode + i * 16 + j) <= 0)
            {
                printf("%d, %d\n", i, j);
                printf("break\n");
                return 1;
            }
    return 0;
}

int main()
{
    double *elas_mode = (double *)malloc(66655 * 16 * sizeof(double));
    memset(elas_mode, 0, 66655 * 16 * sizeof(double)); // initialize elas_mode

    FILE *fp = fopen("elas_mode.txt", "r");

    if (read_my_mode(fp, elas_mode))
        printf("return to main\n");
    
    for (int j = 0; j < 16; j++) // print variables
        if (j == 0)
            printf("%f \t", elas_mode[66654 * 16 + j]);
        else
            printf("%6.5e \t", elas_mode[66654 * 16 + j]);

    fclose(fp);
    free(elas_mode);
    return 0;
}

