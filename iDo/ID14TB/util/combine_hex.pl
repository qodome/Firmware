use strict;
use warnings;

if ($#ARGV != 2) {
    print "Usage: perl combine_hex.pl <bootloader.hex> <app.hex> <combined.hex>\n";
    exit;
}

chdir "../util";

my $cmd_str = "C:/_Software_/util/hexmate \"$ARGV[0]\" \"$ARGV[1]\" -O\"$ARGV[2]\"";
`$cmd_str`;

open FILE, "< $ARGV[1]" or die $!;
my $ext_linear_addr = "";
foreach my $line (<FILE>) {
    if ($line =~ /^:04/) {
        $ext_linear_addr = $line;
        last;
    }
}
close FILE;
if ($ext_linear_addr eq "") {
    die "Cannot get extended linear address from $ARGV[0]";
}
my $output_str = "";
open FILE, "< $ARGV[2]" or die $!;
foreach my $line (<FILE>) {
    if ($line =~ /^:00000001FF/) {
        $output_str = $output_str.$ext_linear_addr;
        $output_str = $output_str.$line;
    } else {
        $output_str = $output_str.$line;
    }
}
close FILE;

open FILE, "> $ARGV[2]" or die $!;
print FILE $output_str;
close FILE;
exit;
