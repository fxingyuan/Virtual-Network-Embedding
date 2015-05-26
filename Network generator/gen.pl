#! /usr/bin/perl -w

#./mkreq <n>  <split_r> <link_r>  <cpu_r> <topo_r>  <maxD>  <scale> <dir_name>
#./mkreq 2000 50        50        25      10        20      100     r-2000-50-50-25-10-20-100

use strict;
use Getopt::Long;

my $i;
open(OUT, "> split-test-gen.sh");

print OUT "#!/bin/bash\n";

for ($i = 0; $i < 101; $i += 25) {
    print OUT "mkdir ../r-250-$i-50-20-10-5-25\n";
    print OUT "./mkreq 250 $i 50 20 10 5 25 r-250-$i-50-20-10-5-25\n";
}

=comment
for ($i = 0; $i < 11; $i ++) {
    print OUT "cd mkgraph\n";
    print OUT "./mkreq 10 $i 5 5\n";
    print OUT "cd ..\n";
    print OUT "./embed 1000\n";
}
=cut
