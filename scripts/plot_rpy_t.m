function plot_rpy_t(log_folder)
    % Dosya yollarını belirleme
    actual_file_path = fullfile(log_folder, 'states.txt');
    ref_file_path = fullfile(log_folder, 'ref_pose.txt');

    % Verileri okuma
    actual_data = readtable(actual_file_path, 'Delimiter', ',');
    ref_data = readtable(ref_file_path, 'Delimiter', ',');

    % Timestamp ve quaternion verilerini ayırma
    actual_timestamp = actual_data{:, 1};    % 1. kolon: timestamp
    actual_quat_x = actual_data{:, 11};      % 11. kolon: actual_quat_x
    actual_quat_y = actual_data{:, 12};      % 12. kolon: actual_quat_y
    actual_quat_z = actual_data{:, 13};      % 13. kolon: actual_quat_z
    actual_quat_w = actual_data{:, 14};      % 14. kolon: actual_quat_w

    ref_timestamp = ref_data{:, 1};          % 1. kolon: timestamp
    ref_quat_x = ref_data{:, 11};            % 11. kolon: ref_quat_x
    ref_quat_y = ref_data{:, 12};            % 12. kolon: ref_quat_y
    ref_quat_z = ref_data{:, 13};            % 13. kolon: ref_quat_z
    ref_quat_w = ref_data{:, 14};            % 14. kolon: ref_quat_w

    % Timestamp verilerini normalize etme
    if ~isempty(actual_timestamp) && ~isempty(ref_timestamp)
        actual_timestamp = actual_timestamp - actual_timestamp(1); % İlk timestamp'i çıkar
        ref_timestamp = ref_timestamp - ref_timestamp(1);          % İlk timestamp'i çıkar
    end

    % Quaternion to Euler conversion (manual calculations)
    actual_eul = quat_to_eul(actual_quat_w, actual_quat_x, actual_quat_y, actual_quat_z);
    ref_eul = quat_to_eul(ref_quat_w, ref_quat_x, ref_quat_y, ref_quat_z);

    % Eğer veriler doğruysa, subplot grafikleri çizelim
    if ~isempty(actual_timestamp) && ~isempty(ref_timestamp)
        figure;

        % Roll subplot
        subplot(3, 1, 1);
        plot(actual_timestamp, actual_eul(:, 1), 'r', 'LineWidth', 1.5); % Actual roll kırmızı çizgi
        hold on;
        plot(ref_timestamp, ref_eul(:, 1), 'k:', 'LineWidth', 1.5);      % Desired roll siyah kesikli çizgi
        xlabel('Normalized Timestamp (s)');
        ylabel('Roll (rad)');
        title('Actual and Desired Roll');
        legend('Actual Roll', 'Desired Roll');
        grid on;
        hold off;

        % Pitch subplot
        subplot(3, 1, 2);
        plot(actual_timestamp, actual_eul(:, 2), 'r', 'LineWidth', 1.5); % Actual pitch kırmızı çizgi
        hold on;
        plot(ref_timestamp, ref_eul(:, 2), 'k:', 'LineWidth', 1.5);      % Desired pitch siyah kesikli çizgi
        xlabel('Normalized Timestamp (s)');
        ylabel('Pitch (rad)');
        title('Actual and Desired Pitch');
        legend('Actual Pitch', 'Desired Pitch');
        grid on;
        hold off;

        % Yaw subplot
        subplot(3, 1, 3);
        plot(actual_timestamp, actual_eul(:, 3), 'r', 'LineWidth', 1.5); % Actual yaw kırmızı çizgi
        hold on;
        plot(ref_timestamp, ref_eul(:, 3), 'k:', 'LineWidth', 1.5);      % Desired yaw siyah kesikli çizgi
        xlabel('Normalized Timestamp (s)');
        ylabel('Yaw (rad)');
        title('Actual and Desired Yaw');
        legend('Actual Yaw', 'Desired Yaw');
        grid on;
        hold off;

    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end

% Manual quaternion to Euler conversion function
function eul = quat_to_eul(w, x, y, z)
    % Compute roll (x-axis rotation)
    sinr_cosp = 2 * (w .* x + y .* z);
    cosr_cosp = 1 - 2 * (x .* x + y .* y);
    roll = atan2(sinr_cosp, cosr_cosp);

    % Compute pitch (y-axis rotation)
    sinp = 2 * (w .* y - z .* x);
    pitch = asin(sinp);

    % Compute yaw (z-axis rotation)
    siny_cosp = 2 * (w .* z + x .* y);
    cosy_cosp = 1 - 2 * (y .* y + z .* z);
    yaw = atan2(siny_cosp, cosy_cosp);

    % Combine into a single matrix
    eul = [roll, pitch, yaw];
end
