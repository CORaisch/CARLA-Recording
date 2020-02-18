#!/home/claudio/Apps/anaconda3/bin/python

# sample call: ./import_recordings_deepvo.py /home/claudio/Datasets/CARLA/sequence_00 DeepVO --host claudio@134.2.178.201

import os, glob, argparse
from shutil import copy, copyfile

ZPAD = 2

def copy_files_local(src_dir, images_dst_path, poses_file_dst):
    # copy poses
    poses_file_src = os.path.join(src_dir, 'poses.txt')
    print('\tcopy poses file \'{}\' to \'{}\''.format(poses_file_src, poses_file_dst))
    copyfile(poses_file_src, poses_file_dst)

    # copy images
    images_src_path = os.path.join(src_dir, 'rgb/left')
    images_src = glob.glob('{}/*'.format(images_src_path))
    for i, im_src in enumerate(images_src):
        print('\tcopy image {}/{}'.format(i, len(images_src)), end='\r')
        copy(im_src, images_dst_path)

def copy_files_remote(host, user, src_dir, images_dst_path, poses_file_dst):
    # connect to host
    import paramiko
    from scp import SCPClient
    ssh = paramiko.SSHClient()
    ssh.load_system_host_keys()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, 22, user, 'gnusmas')
    scp = SCPClient(ssh.get_transport())

    # download poses
    poses_file_src = os.path.join(src_dir, 'poses.txt')
    print('\tcopy poses file \'{}@{}:{}\' to \'{}\''.format(user, host, poses_file_src, poses_file_dst))
    scp.get(poses_file_src, poses_file_dst)

    # download images
    images_src_path = os.path.join(src_dir, 'rgb/left')
    _, stdout, _ = ssh.exec_command('ls -- {}'.format(images_src_path))
    images_src = [ im.split('\n')[0] for im in stdout.readlines() ]
    for i, im_src in enumerate(images_src):
        print('\tcopy image {}/{}'.format(i, len(images_src)), end='\r')
        scp.get(os.path.join(images_src_path, im_src), images_dst_path)
    print()
    scp.close()
    ssh.close()

# setup and parse args
argparser = argparse.ArgumentParser(description="Imports CARLA recordings into directory structure required by DeepVO")
argparser.add_argument('src', type=str, help="Base directory from where data will be loaded")
argparser.add_argument('dst', type=str, help="Base directory to where data will be loaded")
argparser.add_argument('--host', '-host', type=str, default=None, help="set to USER@HOST if src directory is on remote machine with SSH/SCP access enabled")
argparser.add_argument('--recursive', '-r', action='store_true', help="import multiple sequences from all immediate subdirectories of 'src'")
argparser.add_argument('--from_list', '-l', type=str, nargs='+', help="import multiple sequences from all directories passed with this argument and relative to 'src'")
args = argparser.parse_args()

# make list of all directories to import sequences from
if not args.recursive and not args.from_list: # case: import only one sequence from args.src
    srcs = [ args.src ]
elif not args.recursive and args.from_list: # case: import all sequences give in list relative to 'src'
    srcs = [ os.path.join(args.src, d) for d in args.from_list ]
elif args.recursive and not args.from_list: # case: import sequences from all immediate subdirectories of 'src'
    srcs = list(next(os.walk(args.src))[1])
else:
    print("[ERROR] specify at most one option out of [-l, -r]")
    exit()

# make basepath variables
images_dst_base = os.path.join(args.dst, 'images')
poses_dst_base = os.path.join(args.dst, 'poses_gt')

# check if base directory structure already exists
if not os.path.isdir(images_dst_base): os.makedirs(images_dst_base)
if not os.path.isdir(poses_dst_base): os.makedirs(poses_dst_base)

# get consistent idx to save data at
ids_used = [ int(p.split('/')[-1].split('.')[0]) for p in glob.glob('{}/*.txt'.format(poses_dst_base)) ]
if ids_used: # case: already saved sequences at 'dst', get free idcs
    max_id = max(ids_used)
    ids_free = list(set(range(max_id+1)) - set(ids_used))
else: # case: no sequences saved yet at 'dst'
    max_id = -1
    ids_free = []
if len(ids_free) >= len(srcs): # case: more free idcs available than required
    idcs = ids_free[:len(srcs)]
else: # case: less idcs available than required
    idcs = ids_free + [ i for i in range(max_id+1, max_id + 1 + len(srcs)-len(ids_free)) ]
idcs = [ str(idx).zfill(ZPAD) for idx in idcs ]
assert len(idcs)==len(srcs)

# loop and import all sequences
for i, src in enumerate(srcs):
    # make subdirectories
    poses_file_dst = '{}.txt'.format(os.path.join(poses_dst_base, idcs[i]))
    images_dst_path = os.path.join(images_dst_base, idcs[i])
    assert not os.path.isdir(images_dst_path), "images path already exists, script terminated for savety reasons, please resolve situation by hand"
    os.makedirs(images_dst_path)

    # check if src is remote
    if args.host: # case: download files from remote machine
        print("import sequence from '{}:{}' to '{}' at index {}:".format(args.host, src, args.dst, idcs[i]))
        copy_files_remote(args.host.split('@')[1], args.host.split('@')[0], src, images_dst_path, poses_file_dst)
    else: # case: copy files from local machine
        print("import sequence from '{}' to '{}' at index {}:".format(src, args.dst, idcs[i]))
        copy_files_local(src, images_dst_path, poses_file_dst)
