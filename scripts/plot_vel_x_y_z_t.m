function plot_vel_x_y_z_t(log_folder)
    % Dosya yollarını belirleme
    actual_file_path = fullfile(log_folder, 'states.txt');
    ref_file_path = fullfile(log_folder, 'ref_pose.txt');

    % Verileri okuma
    actual_data = readtable(actual_file_path, 'Delimiter', ',');
    ref_data = readtable(ref_file_path, 'Delimiter', ',');

    % Timestamp ve ivme verilerini ayırma
    actual_timestamp = actual_data{:, 1};  % 1. kolon: timestamp
    actual_vel_x = actual_data{:, 5};      % 8. kolon: actual_vel_x
    actual_vel_y = actual_data{:, 6};      % 9. kolon: actual_vel_y
    actual_vel_z = actual_data{:, 7};     % 10. kolon: actual_vel_z

    ref_timestamp = ref_data{:, 1};        % 1. kolon: timestamp
    ref_vel_x = ref_data{:, 5};            % 8. kolon: ref_vel_x
    ref_vel_y = ref_data{:, 6};            % 9. kolon: ref_vel_y
    ref_vel_z = ref_data{:, 7};           % 10. kolon: ref_vel_z

    % Timestamp verilerini normalize etme
    if ~isempty(actual_timestamp) && ~isempty(ref_timestamp)
        actual_timestamp = actual_timestamp - actual_timestamp(1); % İlk timestamp'i çıkar
        ref_timestamp = ref_timestamp - ref_timestamp(1);          % İlk timestamp'i çıkar
    end

    % Eğer veriler doğruysa, subplot grafikleri çizelim
    if ~isempty(actual_timestamp) && ~isempty(actual_vel_x) && ~isempty(actual_vel_y) && ~isempty(actual_vel_z) ...
            && ~isempty(ref_timestamp) && ~isempty(ref_vel_x) && ~isempty(ref_vel_y) && ~isempty(ref_vel_z)
        
        figure;

        % İlk subplot: a_acc_x ve ref_vel_x vs. Normalized Timestamp
        subplot(3, 1, 1);
        plot(actual_timestamp, actual_vel_x, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_vel_x, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Velocity x (m/s)');
        title('Desired and Actual Velocity-Time');
        legend('Actual Vel. x', 'Desired Vel. x');
        grid on;
        hold off;

        % İkinci subplot: a_acc_y ve ref_vel_y vs. Normalized Timestamp
        subplot(3, 1, 2);
        plot(actual_timestamp, actual_vel_y, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_vel_y, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Velocity y (m/s)');
        title('Desired and Actual Velocity-Time');
        legend('Actual Vel. y', 'Desired Vel. y');
        grid on;
        hold off;

        % Üçüncü subplot: a_acc_z ve ref_vel_z vs. Normalized Timestamp
        subplot(3, 1, 3);
        plot(actual_timestamp, actual_vel_z, 'r', 'LineWidth', 1.5);
        hold on;
        plot(ref_timestamp, ref_vel_z, 'k:', 'LineWidth', 1.5);
        xlabel('Time (s)');
        ylabel('Velocity z (m/s)');
        title('Desired and Actual Velocity-Time');
        legend('Actual Vel. z', 'Desired Vel. z');
        grid on;
        hold off;

    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end
