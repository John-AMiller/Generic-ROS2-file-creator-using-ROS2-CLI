from ros2cli.command import CommandExtension
import sys
import subprocess

class CreateCommand(CommandExtension):

    def add_arguments(self, parser, cli_name): #adds arguments that will be used on the CLI
        parser.add_argument(
            'package_name', 
            help='The name of the package where the node will be created.'
        )
        parser.add_argument(
            'node_name',
            help='The name of the node to create.'
        )
        parser.add_argument(
            'language',
            choices=['cpp', 'py'],
            help='The programming language for the node (cpp or py).'
        )

    def main(self, *, args):
        subprocess.run([sys.executable, '-m', 'cli_plugin.create_node', args.package_name, args.node_name, args.language])
