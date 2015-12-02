import argparse
ap = argparse.ArgumentParser()

ap.add_argument('-l', '--learn', required=False, action='store_true')
args = vars(ap.parse_args())

print args['learn']
