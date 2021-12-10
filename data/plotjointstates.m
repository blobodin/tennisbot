function  [t, pos, vel] = plotjointstates(bagfilename, joint)
%
%   [t, pos, vel] = plotjointstates(bagfilename, joint)
%
%   Plot the /joint_states topic saved in the bag file.  If
%   'bagfilename' is not given or given as 'latest', use the most
%   recent bag file.  If 'joint' is given and not 'all', plot only the
%   joint given by an index (number) or name (string).
%
%   Also return the time/pos data, with each column representing one
%   sample-time, each row being a joint.
%

%
%   Check the arguments
%
% If no bagfile is specified, use the most recent.
if (~exist('bagfilename') || strcmp(bagfilename, 'latest'))
    bagfilename = latestbagfilename();
end

% If no joint is given, use all joints.
if (~exist('joint'))
    joint = 'all';
end

%
%   Read the data.
%
msgs = rosbagmsgs(bagfilename, '/seven_arm/joint_states');
[t, pos, vel, eff, names] = jointstatedata(msgs);

%
%   Potentially isolate a single joint.
%
if (exist('joint') && (~strcmp(joint, 'all')))
    % Check the number of joints (by data).
    Nnam = length(names);
    Npos = size(pos, 1);
    Nvel = size(vel, 1);
    Neff = size(eff, 1);
    Nmax = max([Nnam, Npos, Nvel, Neff]);

    % For a numeric joint specification.
    if (isnumeric(joint))
        if (numel(joint) ~= 1)
            error('Bad joint index specification');
        end
            
        % Grab the joint number and use as index.
        ind = floor(joint);
        if ((ind < 1) || (ind > Nmax))
            error(['Out of range joint index ' num2str(ind)]);
        end
        disp(['Using only joint index ' num2str(ind)]);

    % For a joint name specification.
    elseif (ischar(joint))
        % Find the index for the joint name.
        ind = find(strcmp(names, joint));
        if (isempty(ind))
            error(['Unable to isolate joint ''' joint ''''])
        end
        disp(['Using only joint ''' joint '''']);

    % Otherwise can't do anything.
    else
        error('Bad joint argument');
    end
    
    % Isolate the data.
    if (ind <= Nnam) names = names(ind); else names = {}; end
    if (ind <= Npos) pos   = pos(ind,:); else pos   = []; end
    if (ind <= Nvel) vel   = vel(ind,:); else vel   = []; end
    if (ind <= Neff) eff   = eff(ind,:); else eff   = []; end
end

%
%   Plot.
%
% Skip if outputing data.
if (nargout)
    disp('Skipping the plot when outputing data.');
    return;
end

% Prepare the figure.
figure(gcf);
clf;

% Plot.
ax(1) = subplot(2,1,1);
plot(t, pos, 'LineWidth', 2);
grid on;
ylabel('Position (rad)');
title(bagfilename);
legend(names);

ax(2) = subplot(2,1,2);
plot(t, vel, 'Linewidth', 2);
grid on;
ylabel('Velocity (rad/sec)');
xlabel('Time (sec)');

linkaxes(ax, 'x');

% Name the Figure and span the full 8.5x11 page.
set(gcf, 'Name',          'Joint Data');
set(gcf, 'PaperPosition', [0.25 0.25 8.00 10.50]);

% Return to the top subplot, so subsequent title()'s go here...
subplot(2,1,1);

end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function bagfilename = latestbagfilename()
%
%   bagfilename = latestbagfilename()
%
%   Return the name of the latest bag file.  Error if there are not
%   bag files.
%

% Get a list of .bag files in the current folder.
d = dir('*.bag');

% Make sure we have at least one bag file.
if (~size(d,1))
    error('Unable to find a bag file');
end

% Find the most recently modified file
[~, idx] = max([d.datenum]);

% Use as the bag file.
bagfilename = d(idx).name;
disp(['Using bag file ''' bagfilename '''']);

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  msgs = rosbagmsgs(bagfilename, topicname)
%
%   msgs = rosbagmsgs(bagfilename, topicname)
%
%   Extract the messages of the named topic from the bagfile.  The
%   messages are returned as a struct array.  The structure contains
%   MessageType as well as the fields of the topic.
%

% Load the bag.
try
    bag = rosbag(bagfilename);
catch
    error(['Unable to open the bag file ''' bagfilename '''']);
end

% Isolate the specified topic.
topic = select(bag, 'Topic', topicname);
if (~topic.NumMessages)
    warning(['No messages under topic ''' topicname '''']);
end

% Convert the messages in the topic into structure array.
msgs = cell2mat(readMessages(topic, 'DataFormat', 'struct'));

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  [t, pos, vel, eff, names] = jointstatedata(msgs)
%
%   [t, pos, vel, eff, names] = jointstatedata(msgs)
%
%   Extract the data from the given JointStates messages.  Time is
%   shifted to start at zero for the first message.  The return data
%   gives a column per time sample, and a row per joint (assuming data
%   is available).
%

% Double-check the type.
if (~strcmp(msgs(1).MessageType, 'sensor_msgs/JointState'))
    error(['Messages are not of type sensor_msgs/JointState']);
end

% Extract the names of the joints and double-check.
names = msgs(1).Name;
if (length(names) == 0)
    error(['Messages contain no joints']);
end

% Extract the time (converting from sec/nsec).
headers = [msgs(:).Header];
stamps  = [headers(:).Stamp];

sec  = double([stamps(:).Sec]);
nsec = double([stamps(:).Nsec]);

t = (sec - sec(1)) + 1e-9 * (nsec - nsec(1));

% Extract the msgs.
pos = double([msgs(:).Position]);
vel = double([msgs(:).Velocity]);
eff = double([msgs(:).Effort]);

end
