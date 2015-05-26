#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include "def.h"

#define LEN_CMD 128

#define POISSON_MEAN 25
//#define POISSON_MEAN 50
#define TOTAL_TIME 50000

#define RAND_MAX 2147483647

//#define STAR_NODES_MOST 4

//#define LONG_REQ_RATE    0.2
//#define SHORT_REQ_RATE   0.8

#define REQ_DURATION        1000
#define MIN_REQ_DURATION    250

#define MIN_NUM_NODE    2
#define AVG_NUM_NODE    5
#define LARGE_NUM_NODE   10

//#define SPLITTABLE_REQ_RATE 0.8
//#define UNSPLITTABLE_REQ_RATE 0.2

//#define TOPO_GENERAL_RATE 0.4
//#define TOPO_STAR_RATE 0.6

#define MIGRATION_RATE 0.9
#define NO_MIGRATION_RATE 0.1

double dis(int x1, int y1, int x2, int y2) {
  return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

int poisson(double lambda) {
  double p;
  int r;
  p = 0;
  r = 0;

  while (1) {
    p = p - log(rand() / (double)RAND_MAX);
    if (p < lambda) {
      r++;
    } else {
      break;
    }
  }
  
  return r;
}

int main(int argc, char **argv) {
  int xseed = (unsigned)time(NULL);  // Current Time
  srand( xseed ); 

  if (argc != 9) {
    printf("mkreq <n> <split_r> <link_r> <cpu_r> <topo_r> <maxD> <scale> <dir_name>\n");
    exit(1);
  }

  //USAGE: mkreq <#req> [splittable rate] [link rate]
  int n = atoi(argv[1]);  // number of graphs
  double splittable_rate = atoi(argv[2])/(double)100;  // % of requests that support path splitting
  double link_rate = atoi(argv[3])/(double)100;  // bound max bw
  double cpu_rate = atoi(argv[4])/(double)100;  // bound max cpu
  double topo_general_rate = atoi(argv[5])/(double)10;
  int MAX_DISTANCE = atoi(argv[6]);
  //dir argv[5]
  //double node_rate = atoi(argv[5])/(double)10;
  printf("split %lf, link %lf topo_gen %lf\n", splittable_rate, link_rate, topo_general_rate);
  
  FILE * fp;
  FILE * reqfile;
  char filename[LEN_FILENAME], reqfilename[LEN_FILENAME];
  char cmd[LEN_CMD];
  int i, k = 0, countk = 0, p = 0, start;
  int *n_x, *n_y, t1, t2;
  int l_c = 0, a_c = 0;
  
  int num_nodes, num_edges, split, time, duration, from, to, topo, maxD = MAX_DISTANCE;

  for (i = 0; i < n; i ++) {
    sprintf(filename, "spec/itm-spec%d", i);
    fp = fopen(filename, "w");
    fprintf(fp, "geo 1\n");
    int t;
    //sleep(2);
    int ran = rand();
    if (((double)ran / (double)RAND_MAX) < 0.25) {
      t = (rand() % (LARGE_NUM_NODE - MIN_NUM_NODE)) + MIN_NUM_NODE;
      l_c++;
    } else {
      t = (rand() % (AVG_NUM_NODE - MIN_NUM_NODE)) + MIN_NUM_NODE;      
      a_c++;
    }    
    printf("req%d rand %.2lf, #nodes: %d\n", i, (double)ran / (double)RAND_MAX, t);
    fprintf(fp, "%d %d 2 0.5 0.2\n", t, atoi(argv[7])); // scale: argv[5]; method: 2; alpha: 0.5; beta: 0.2; gamma: default
    //3 - pure random
    fclose(fp);
  }

  sleep(1);

  for (i = 0; i < n; i ++) {
    sprintf(cmd, "./itm spec/itm-spec%d",i);
    //printf("%s\n", cmd);
    system(cmd);
  }

  for (i = 0; i < n; i ++) {
    sprintf(cmd, "./sgb2alt spec/itm-spec%d-0.gb alt/%d.alt", i, i);
    //printf("%s\n", cmd);
    system(cmd);
  }
   
  char str[1000];
  int j;
  for (i = 0; i < n; i ++) {
    printf("generate req %d\n", i);
    sprintf(filename, "alt/%d.alt", i);
    fp = fopen(filename, "r");
    sprintf(reqfilename, "../%s/req%d.txt", argv[8], i);
    reqfile = fopen(reqfilename, "w");
    if (reqfile == NULL) {
      printf("couldn't open file %s\n", reqfilename);
      exit(1);
    }

    for (j = 0; j < 10; j ++)
      fscanf(fp, "%s", str);

    fscanf(fp, "%d %d %*d %*d", &num_nodes, &num_edges);
    num_edges /= 2;

    double X;

    if ((X = rand()/(double)RAND_MAX) < splittable_rate) {
      split = LINK_SPLITTABLE;
    } else {
      split = LINK_UNSPLITTABLE;
    }

    printf ("X=%.2lf SR=%.2lf\n", X, splittable_rate);

    if (countk == k) {
      k = 0;
      while( k == 0) {
        k = poisson(POISSON_MEAN);
      }
      countk = 0;
      printf("k %d\n", k);
      start = (p * TOTAL_TIME * POISSON_MEAN) / n;
      p++; 
    }

    time = start + ((countk + 1) * TOTAL_TIME * POISSON_MEAN) / (n * (k + 1));
    countk ++;

    /*if (rand()/(double)RAND_MAX < LONG_REQ_RATE) {
      duration = LONG_REQ_DURATION;
    } else {
      duration = SHORT_REQ_DURATION;
      }*/
    duration = MIN_REQ_DURATION + (int)(-log(rand() / (double)RAND_MAX) * (REQ_DURATION - MIN_REQ_DURATION)); // exponentially distributed duration

    if (rand()/(double)RAND_MAX < topo_general_rate) {
      topo = TOPO_GENERAL;
    } else {
      topo = TOPO_STAR;
      //num_nodes = (int)((rand()/(double)RAND_MAX) * STAR_NODES_MOST) + 1;
      num_edges = num_nodes - 1;
    }

    fprintf(reqfile, "%d %d %d %d %d %d %d\n", num_nodes, num_edges, split, time, duration, topo, maxD);
    printf("nodes %d, edges %d\n", num_nodes, num_edges);
    printf("time %d, duration %d\n", time, duration);

    // skip
    for (j = 0; j < 11; j ++) {
      fscanf(fp, "%s", str);
      //printf("%s\n", str);
    }

    n_x = (int *)malloc(num_nodes * sizeof(int));
    n_y = (int *)malloc(num_nodes * sizeof(int));

    for (j = 0; j < num_nodes; j ++) {
      fscanf(fp, "%d %d %d %d", &t1, &t2, &n_x[j], &n_y[j]);
      //printf("%d %d %d %d\n", t1, t2, n_x[j], n_y[j]);      
      double t = rand();      
      fprintf(reqfile, "%d %d %lf\n", n_x[j], n_y[j], t / (double)RAND_MAX * (double)MAX_CPU * cpu_rate);      
    }
    
    // skip
    for (j = 0; j < 6; j ++) {
      fscanf(fp, "%s", str);
      //printf("%s\n", str);
    }       

    for (j = 0; j < num_edges; j ++) {
      fscanf(fp, "%d %d %*d %*d", &from, &to);
      if (topo == TOPO_GENERAL) 
        fprintf(reqfile, "%d %d %lf %lf\n", from, to, rand()/(double)RAND_MAX * (double)MAX_BW * link_rate, (1. + rand()/(double)RAND_MAX) * dis(n_x[from], n_y[from], n_x[to], n_y[to])); 
    }

    if (topo == TOPO_STAR) {
      for (j = 0; j < num_nodes - 1; j ++) {
        fprintf(reqfile, "%d %d %lf %lf\n", j, num_nodes-1, rand()/(double)RAND_MAX * (double)MAX_BW * link_rate, (1. + rand()/(double)RAND_MAX) * dis(n_x[from], n_y[from], n_x[to], n_y[to])); 
      }
    }

    free(n_x);
    free(n_y);

    fclose(fp);
    fclose(reqfile);
  }
  
  return 0;
}

