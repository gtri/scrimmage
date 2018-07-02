"""Copies csv files after a scrimmage run to a postgres database."""
import argparse
import os.path
import glob
#  import pdb
import psycopg2
from psycopg2 import sql
#  from psycopg2 import sql

# We need a log directory and datatypes fed to us
# example:
# python csv_to_postgres.py ~/.scrimmage/logs/2018-06-28_09-41-11/ "DOUBLE
# PRECISION" "TEXT" "INT"
parser = argparse.ArgumentParser(
    description='''Moves a the .csv files from a scrimmage log folder a
    postgres database''')
parser.add_argument("log_dir", help='The log directory, for example ~/.scrimmage/logs/2018-06-28_09-41-11/')
parser.add_argument("data_types", nargs="*", help='We assume all TEXT datatypes for now while we architect a better solution for inputting datatypes')
args = parser.parse_args()
#  args = parser.parse_known_args(["log_dir"])
log_dir = args.log_dir
data_types = args.data_types
#  print(args.log_dir)

# Connect to existing database
connect_string = "dbname=scrimmage user=scrimmage password=scrimmage"
files = glob.glob(log_dir + "*.csv")
if len(files) == 0:
    print("No csv files found, exiting without doing creating anything in the"
          " database")
    exit()
with psycopg2.connect(connect_string) as conn:

    # Open cursor to perform operations
    with conn.cursor() as cur:
        cur = conn.cursor()
        # Assume we have the log directory put through

        # Assume we pass through tables, columns, datatypes that we want
        # Build the table
        #  log_dir = '~/.scrimmage/logs/2018-06-28_09-41-11/'
        timestamp = os.path.basename(os.path.dirname(log_dir))
        timestamp = timestamp.replace('-', '_')
        schema = 's' + timestamp
        cur.execute('CREATE SCHEMA IF NOT EXISTS ' + schema + ";")
        # conn.commit() gets called automatically when using with syntax
        # conn.commit()

    # Assume we have a list of tables that we want to make? Or can glob for
    # csvs in the log directory
    #  data_types = [
    #  "INT PRIMARY KEY", "DOUBLE PRECISION",
    #  "DOUBLE PRECISION", "DOUBLE PRECISION", "DOUBLE PRECISION",
    #  "DOUBLE PRECISION", "DOUBLE PRECISION", "DOUBLE PRECISION"]

    #  query = 'CREATE TABLE ' + schema + "." + table_name + "( "
    # for (col, data_type) in zip(columns, data_types):

    # Actually copy all of the data
    for fi in files:
        with open(fi, 'r') as f:
            # Notice that we don't need the `csv` module.
            # next(f)  # Skip the header row.
            table = os.path.basename(os.path.splitext(fi)[0])
            full_table = "{}.{}".format(schema, table)
            cols = f.readline()
            cols = cols.splitlines()[0]
            cols = cols.split(',')
            # could get datatypes with
            if len(data_types) != len(cols):
                data_types = ["TEXT" for i in range(0, len(cols))]
                print("Not enough Postgres datatypes specified for {} table,"
                      " defaulting to all TEXT".format(table))

            # ["DOUBLE PRECISION" for i in range(0, len(cols))]
            query_s = sql.SQL('CREATE TABLE {}').format(sql.SQL(full_table))
            to_join = ''.join([col + ' ' + data_type + ', ' for col, data_type
                               in zip(cols, data_types)])
            to_join = to_join[:-2]
            to_join_sql = sql.SQL("({});").format(sql.SQL(to_join))
            query = query_s + to_join_sql
            #  query = sql.SQL(query_string)
            with conn.cursor() as cur:
                res = cur.execute(query)

                cur.copy_from(f, "{}".format(full_table), sep=',')
                # conn.commit()
conn.close()
